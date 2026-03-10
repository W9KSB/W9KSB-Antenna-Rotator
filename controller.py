# controller.py
import time
import threading
import atexit

import config
import position
import movement

from Adafruit_MotorHAT import Adafruit_MotorHAT


def clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


class RotatorController:
    """
    Owns hardware and runs a background control loop.
    Exposes:
      - set_target(az_deg, el_deg)
      - get_position() -> (az_deg, el_deg)
      - stop()
    """

    def __init__(self, debug=False):
        self.debug = debug

        # Calibration
        self.cal = config.load_cal()

        # Motors
        self.mh = movement.init_motorhat()
        self.motor_az = self.mh.getMotor(1)
        self.motor_el = self.mh.getMotor(2)

        # Encoders
        self.enc_az = position.AS5600(config.ENCODER1_BUS, name="E1")
        self.enc_el = position.AS5600(config.ENCODER2_BUS, name="E2")
        self.az_tracker = position.UnwrappedAngle(self.enc_az)

        # State
        self._lock = threading.Lock()
        self._target_az = None  # physical 0..360
        self._target_el = None  # physical 0..90
        self._stop_requested = False

        self._cur_az_phys = 0.0
        self._cur_el_phys = 0.0
        self._cur_el_raw = 0.0  # raw AS5600 degrees (0..360)

        # Arrival reporting (print once per target)
        self._arrived_reported = False
        self._last_arrival_target = None  # (az, el)

        self._thread = None
        self._running = False

        # ----------------------------
        # EL control: breakaway search + approach slow + stall re-kick
        # ----------------------------
        self._el_dir_change_settle_s = float(getattr(config, "EL_DIR_CHANGE_SETTLE_S", 0.25))

        # Movement detection (raw degrees)
        self._el_move_thresh_deg = float(getattr(config, "EL_MOVE_THRESH_DEG", 0.25))
        self._el_move_window_s = float(getattr(config, "EL_MOVE_WINDOW_S", 0.30))

        # Stall detection (no meaningful movement while commanding)
        self._el_stall_time_s = float(getattr(config, "EL_STALL_TIME_S", 0.45))

        # Breakaway search (UP)
        self._el_up_bk_start = int(getattr(config, "EL_UP_BREAKAWAY_START_SPEED", 150))
        self._el_up_bk_step = int(getattr(config, "EL_UP_BREAKAWAY_STEP_SPEED", 25))
        self._el_up_bk_max = int(getattr(config, "EL_UP_BREAKAWAY_MAX_SPEED", 255))
        self._el_up_bk_interval_s = float(getattr(config, "EL_UP_BREAKAWAY_INTERVAL_S", 0.30))

        # Breakaway search (DOWN)
        self._el_down_bk_start = int(getattr(config, "EL_DOWN_BREAKAWAY_START_SPEED", 150))
        self._el_down_bk_step = int(getattr(config, "EL_DOWN_BREAKAWAY_STEP_SPEED", 25))
        self._el_down_bk_max = int(getattr(config, "EL_DOWN_BREAKAWAY_MAX_SPEED", 255))
        self._el_down_bk_interval_s = float(getattr(config, "EL_DOWN_BREAKAWAY_INTERVAL_S", 0.30))

        # Approach/creep windows
        self._el_approach_window_deg = float(getattr(config, "EL_APPROACH_WINDOW_DEG", 8.0))
        self._el_creep_window_deg = float(getattr(config, "EL_CREEP_WINDOW_DEG", 2.5))

        # Fixed approach/creep speeds
        self._el_approach_speed = int(getattr(config, "EL_APPROACH_SPEED", 120))
        self._el_creep_speed = int(getattr(config, "EL_CREEP_SPEED", 80))

        # UP travel uses your existing schedule (already in config)
        self._el_up_break_1 = float(getattr(config, "EL_UP_BREAK_1_DEG", 25.0))
        self._el_up_break_2 = float(getattr(config, "EL_UP_BREAK_2_DEG", 45.0))
        self._el_up_speed_0_25 = int(getattr(config, "EL_UP_SPEED_0_25", 250))
        self._el_up_speed_25_45 = int(getattr(config, "EL_UP_SPEED_25_45", 200))
        self._el_up_speed_45_max = int(getattr(config, "EL_UP_SPEED_45_MAX", 125))

        # DOWN travel base (used after breakaway, before approach windows)
        # (You can set this in config later; default keeps it strong but not max.)
        self._el_down_travel_speed = int(getattr(config, "EL_DOWN_TRAVEL_SPEED", 200))

        # Internal EL runtime
        self._el_state = "IDLE"   # IDLE, UP_BREAKAWAY, DOWN_BREAKAWAY, RUN
        self._el_last_dir = 0     # -1, 0, +1
        self._el_cmd_speed = 0
        self._el_next_step_ts = 0.0

        # Movement/stall bookkeeping
        self._el_last_move_raw = None
        self._el_last_move_ts = None
        self._el_stall_start_ts = None

        atexit.register(self.shutdown)

        # Prime readings + auto-zero AZ session-only
        try:
            _ = position.stable_read_unwrapped(self.az_tracker, seconds=0.25)
            _ = position.stable_read_wrapped(self.enc_el, seconds=0.25)

            cur_az_unwrapped = position.stable_read_unwrapped(self.az_tracker, seconds=0.25)
            self.cal["az_offset_deg"] = cur_az_unwrapped % 360.0
            print(f"[AZ] Auto-zero on startup: az_offset_deg={self.cal['az_offset_deg']:.6f}", flush=True)

            self._update_current_position()
        except Exception:
            pass

    # ---------------------------
    # Public API
    # ---------------------------

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        try:
            self.stop()
        except Exception:
            pass

    def stop(self):
        with self._lock:
            self._stop_requested = True
            self._target_az = None
            self._target_el = None
            self._arrived_reported = False
            self._last_arrival_target = None

        self._el_reset()
        movement.stop_motor(self.motor_az)
        movement.stop_motor(self.motor_el)

    def set_target(self, az_deg, el_deg):
        az = float(az_deg) % 360.0
        el = clamp(el_deg, config.EL_MIN_DEG, config.EL_MAX_DEG)
        with self._lock:
            self._target_az = az
            self._target_el = el
            self._stop_requested = False
            self._arrived_reported = False
            self._last_arrival_target = (az, el)

    def get_position(self):
        with self._lock:
            return (float(self._cur_az_phys), float(self._cur_el_phys))

    def set_az_home_here(self):
        cur_unwrapped = position.stable_read_unwrapped(self.az_tracker, seconds=0.25)
        self.cal["az_offset_deg"] = cur_unwrapped % 360.0
        config.save_cal(self.cal)

    def set_el_zero_here(self):
        cur_raw = position.stable_read_wrapped(self.enc_el, seconds=0.25)
        self.cal["el_offset_deg"] = cur_raw % 360.0
        config.save_cal(self.cal)

    # ---------------------------
    # Internals
    # ---------------------------

    def _log(self, msg: str):
        if self.debug:
            print(msg, flush=True)

    def _motor_el_set(self, signed_dir: int, speed_0_255: int):
        if signed_dir == 0 or speed_0_255 <= 0:
            try:
                self.motor_el.setSpeed(0)
            except Exception:
                pass
            self.motor_el.run(Adafruit_MotorHAT.RELEASE)
            return

        if (signed_dir * int(config.M2_FORWARD_SIGN)) > 0:
            self.motor_el.run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motor_el.run(Adafruit_MotorHAT.BACKWARD)

        self.motor_el.setSpeed(int(clamp(speed_0_255, 0, 255)))

    def _wrap_delta_deg(self, cur_raw: float, prev_raw: float) -> float:
        return (cur_raw - prev_raw + 180.0) % 360.0 - 180.0

    def _el_reset(self):
        self._el_state = "IDLE"
        self._el_last_dir = 0
        self._el_cmd_speed = 0
        self._el_next_step_ts = 0.0

        self._el_last_move_raw = None
        self._el_last_move_ts = None
        self._el_stall_start_ts = None

    def _el_mark_move(self, cur_raw: float):
        self._el_last_move_raw = cur_raw
        self._el_last_move_ts = time.time()
        self._el_stall_start_ts = None

    def _el_moved_recently(self, cur_raw: float) -> bool:
        now = time.time()
        if self._el_last_move_raw is None or self._el_last_move_ts is None:
            self._el_last_move_raw = cur_raw
            self._el_last_move_ts = now
            return False

        if (now - self._el_last_move_ts) < 0.01:
            return False

        if (now - self._el_last_move_ts) >= self._el_move_window_s:
            d = self._wrap_delta_deg(cur_raw, self._el_last_move_raw)
            if abs(d) >= self._el_move_thresh_deg:
                self._el_mark_move(cur_raw)
                return True
            else:
                # refresh window anchor (prevents old anchor from causing weird deltas)
                self._el_last_move_raw = cur_raw
                self._el_last_move_ts = now
                return False

        return False

    def _el_stalled(self) -> bool:
        now = time.time()
        if self._el_stall_start_ts is None:
            self._el_stall_start_ts = now
            return False
        return (now - self._el_stall_start_ts) >= self._el_stall_time_s

    def _el_up_schedule_speed(self, cur_el_phys: float) -> int:
        if cur_el_phys < self._el_up_break_1:
            return self._el_up_speed_0_25
        if cur_el_phys < self._el_up_break_2:
            return self._el_up_speed_25_45
        return self._el_up_speed_45_max

    def _el_enter_breakaway(self, desired_dir: int, cur_raw: float):
        now = time.time()
        if desired_dir > 0:
            self._el_state = "UP_BREAKAWAY"
            self._el_cmd_speed = int(clamp(self._el_up_bk_start, 0, 255))
            self._el_next_step_ts = now + self._el_up_bk_interval_s
        else:
            self._el_state = "DOWN_BREAKAWAY"
            self._el_cmd_speed = int(clamp(self._el_down_bk_start, 0, 255))
            self._el_next_step_ts = now + self._el_down_bk_interval_s

        self._el_last_move_raw = cur_raw
        self._el_last_move_ts = now
        self._el_stall_start_ts = None

        if self.debug:
            self._log(f"[EL] enter {self._el_state} start_speed={self._el_cmd_speed}")

    def _el_tick(self, el_err_deg: float, cur_el_phys: float, cur_raw: float) -> bool:
        # deadband
        if abs(el_err_deg) <= config.DEADBAND_DEG:
            self._motor_el_set(0, 0)
            self._el_reset()
            return True

        desired_dir = +1 if el_err_deg > 0 else -1

        # direction-change settle
        if desired_dir != 0 and self._el_last_dir != 0 and desired_dir != self._el_last_dir:
            if self.debug:
                self._log(f"[EL] dir change {self._el_last_dir} -> {desired_dir}: STOP+SETTLE {self._el_dir_change_settle_s:.2f}s")
            self._motor_el_set(0, 0)
            movement.stop_motor(self.motor_el)
            self._el_reset()
            time.sleep(self._el_dir_change_settle_s)

        # Start breakaway if idle or direction changed / state mismatched
        if self._el_state == "IDLE":
            self._el_enter_breakaway(desired_dir, cur_raw)

        if (desired_dir > 0 and self._el_state == "DOWN_BREAKAWAY") or (desired_dir < 0 and self._el_state == "UP_BREAKAWAY"):
            self._el_enter_breakaway(desired_dir, cur_raw)

        now = time.time()

        # Determine requested speed for approach/creep when RUN
        def approach_speed() -> int:
            aerr = abs(el_err_deg)
            if aerr <= self._el_creep_window_deg:
                return int(clamp(self._el_creep_speed, 0, 255))
            if aerr <= self._el_approach_window_deg:
                return int(clamp(self._el_approach_speed, 0, 255))
            return -1  # not in approach/creep

        # BREAKAWAY modes: step until movement is detected
        if self._el_state in ("UP_BREAKAWAY", "DOWN_BREAKAWAY"):
            # command current breakaway speed
            self._motor_el_set(desired_dir, self._el_cmd_speed)

            moved = self._el_moved_recently(cur_raw)
            if moved:
                # movement detected -> RUN
                self._el_state = "RUN"
                self._el_stall_start_ts = None
                if self.debug:
                    self._log(f"[EL] movement detected -> RUN (bk_speed={self._el_cmd_speed})")
                return True

            # no movement yet: stall timer
            if self._el_stall_start_ts is None:
                self._el_stall_start_ts = now

            # step-up timing
            interval = self._el_up_bk_interval_s if desired_dir > 0 else self._el_down_bk_interval_s
            step = self._el_up_bk_step if desired_dir > 0 else self._el_down_bk_step
            vmax = self._el_up_bk_max if desired_dir > 0 else self._el_down_bk_max

            if now >= self._el_next_step_ts:
                old = self._el_cmd_speed
                self._el_cmd_speed = int(clamp(old + step, 0, vmax))
                self._el_next_step_ts = now + interval
                if self.debug:
                    self._log(f"[EL] breakaway step {old}->{self._el_cmd_speed} (max={vmax})")

            return True

        # RUN mode: choose speed based on approach/creep, but if we stall, re-enter breakaway
        if self._el_state == "RUN":
            sp = approach_speed()

            if sp < 0:
                # not in approach window: travel speed
                if desired_dir > 0:
                    sp = int(clamp(self._el_up_schedule_speed(cur_el_phys), 0, 255))
                else:
                    sp = int(clamp(self._el_down_travel_speed, 0, 255))

            self._motor_el_set(desired_dir, sp)

            # update movement check; if we see movement, clear stall timer
            if self._el_moved_recently(cur_raw):
                return True

            # no movement: start/advance stall timer, then re-kick into breakaway
            if self._el_stall_start_ts is None:
                self._el_stall_start_ts = now
                return True

            if self._el_stalled():
                # re-enter breakaway, but seed start speed from the speed we were using
                if desired_dir > 0:
                    self._el_up_bk_start = int(min(255, max(self._el_up_bk_start, sp)))
                else:
                    self._el_down_bk_start = int(min(255, max(self._el_down_bk_start, sp)))

                if self.debug:
                    self._log(f"[EL] stall in RUN at sp={sp} -> re-breakaway (dir={desired_dir})")

                self._el_enter_breakaway(desired_dir, cur_raw)
                return True

            return True

        return False

    def _update_current_position(self):
        # AZ
        az_unwrapped = self.az_tracker.read()
        az_phys = position.az_unwrapped_to_physical(az_unwrapped, self.cal["az_offset_deg"])

        # EL
        el_raw = self.enc_el.read_degrees_filtered()
        el_phys = position.el_raw_to_physical(el_raw, self.cal["el_offset_deg"])

        with self._lock:
            self._cur_az_phys = az_phys % 360.0
            self._cur_el_phys = el_phys
            self._cur_el_raw = float(el_raw)

    def _loop(self):
        period = 1.0 / float(config.CONTROL_HZ)

        while self._running:
            t0 = time.time()
            try:
                self._update_current_position()

                with self._lock:
                    target_az = self._target_az
                    target_el = self._target_el
                    stop_req = self._stop_requested
                    arrived_reported = self._arrived_reported

                if stop_req or (target_az is None and target_el is None):
                    self._el_reset()
                    movement.stop_motor(self.motor_az)
                    movement.stop_motor(self.motor_el)
                else:
                    cur_az_unwrapped = self.az_tracker.unwrapped
                    cur_az_phys, cur_el_phys = self.get_position()
                    cur_el_raw = float(self._cur_el_raw)

                    tgt_el = clamp(target_el, config.EL_MIN_DEG, config.EL_MAX_DEG)
                    el_err = tgt_el - cur_el_phys

                    tgt_unwrapped = position.nearest_unwrapped_target(
                        current_unwrapped=cur_az_unwrapped,
                        target_az_phys_deg=target_az,
                        az_offset_deg=self.cal["az_offset_deg"],
                    )
                    az_err = tgt_unwrapped - cur_az_unwrapped

                    az_done = abs(az_err) <= config.DEADBAND_DEG
                    el_done = abs(el_err) <= config.DEADBAND_DEG

                    if az_done and el_done:
                        self._el_reset()
                        movement.stop_motor(self.motor_az)
                        movement.stop_motor(self.motor_el)

                        if not arrived_reported:
                            with self._lock:
                                if (self._target_az is not None) and (self._target_el is not None) and (not self._arrived_reported):
                                    az_print = self._cur_az_phys
                                    el_print = self._cur_el_phys
                                    tgt = self._last_arrival_target
                                    self._arrived_reported = True

                                    if tgt is not None:
                                        print(
                                            f"[ARRIVED] AZ={az_print:7.2f}°  EL={el_print:6.2f}°   (target AZ={tgt[0]:.2f} EL={tgt[1]:.2f})",
                                            flush=True
                                        )
                                    else:
                                        print(f"[ARRIVED] AZ={az_print:7.2f}°  EL={el_print:6.2f}°", flush=True)
                    else:
                        if arrived_reported:
                            with self._lock:
                                self._arrived_reported = False

                        # EL safety clamp
                        if (cur_el_phys <= config.EL_MIN_DEG + config.DEADBAND_DEG) and (el_err < 0):
                            self._el_reset()
                            movement.stop_motor(self.motor_el)
                            self._el_last_dir = 0
                        elif (cur_el_phys >= config.EL_MAX_DEG - config.DEADBAND_DEG) and (el_err > 0):
                            self._el_reset()
                            movement.stop_motor(self.motor_el)
                            self._el_last_dir = 0
                        else:
                            handled = self._el_tick(el_err, cur_el_phys, cur_el_raw)
                            if not handled:
                                # fallback to original behavior
                                self._el_reset()
                                movement.drive_toward_error(self.motor_el, config.M2_FORWARD_SIGN, el_err)

                            # record last dir for settle logic
                            if el_err > config.DEADBAND_DEG:
                                self._el_last_dir = +1
                            elif el_err < -config.DEADBAND_DEG:
                                self._el_last_dir = -1
                            else:
                                self._el_last_dir = 0

                        # AZ unchanged
                        movement.drive_toward_error(self.motor_az, config.M1_FORWARD_SIGN, az_err)

            except Exception:
                try:
                    self._el_reset()
                    movement.stop_motor(self.motor_az)
                    movement.stop_motor(self.motor_el)
                except Exception:
                    pass

            dt = time.time() - t0
            sleep_for = period - dt
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Rotator controller core (no Hamlib server).")
    p.add_argument("--status", action="store_true")
    p.add_argument("--az", type=float)
    p.add_argument("--el", type=float)
    p.add_argument("--az_home_here", action="store_true")
    p.add_argument("--el_zero_here", action="store_true")
    p.add_argument("--debug", action="store_true")
    args = p.parse_args()

    rc = RotatorController(debug=args.debug)
    rc.start()

    if args.az_home_here:
        rc.set_az_home_here()
        print("AZ home saved.")
        raise SystemExit(0)

    if args.el_zero_here:
        rc.set_el_zero_here()
        print("EL zero saved.")
        raise SystemExit(0)

    if args.status:
        time.sleep(0.3)
        az, el = rc.get_position()
        print(f"AZ: {az:.2f}° (offset {rc.cal['az_offset_deg']:.2f})   EL: {el:.2f}° (offset {rc.cal['el_offset_deg']:.2f})")
        raise SystemExit(0)

    if args.az is not None or args.el is not None:
        rc.set_target(
            args.az if args.az is not None else 0.0,
            args.el if args.el is not None else 0.0
        )
        while True:
            time.sleep(1.0)
