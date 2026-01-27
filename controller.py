# controller.py
import time
import threading
import atexit

import config
import position
import movement


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

        # Arrival reporting (print once per target)
        self._arrived_reported = False
        self._last_arrival_target = None  # (az, el)

        self._thread = None
        self._running = False

        atexit.register(self.shutdown)

        # Prime readings
        try:
            # Let encoders settle and establish tracker state
            _ = position.stable_read_unwrapped(self.az_tracker, seconds=0.25)
            _ = position.stable_read_wrapped(self.enc_el, seconds=0.25)

            # -----------------------------
            # AUTO-ZERO AZ ON STARTUP (NEW)
            # -----------------------------
            # Wherever AZ is right now becomes AZ=0.00 for this run.
            # Elevation offset is NOT touched.
            cur_az_unwrapped = position.stable_read_unwrapped(self.az_tracker, seconds=0.25)
            self.cal["az_offset_deg"] = cur_az_unwrapped % 360.0
            # NOTE: not writing to rotator_cal.json on purpose (session-only)

            print(f"[AZ] Auto-zero on startup: az_offset_deg={self.cal['az_offset_deg']:.6f}", flush=True)

            # Update cached position fields
            self._update_current_position()

        except Exception:
            # still allow startup; errors will show in loop
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
        movement.stop_motor(self.motor_az)
        movement.stop_motor(self.motor_el)

    def set_target(self, az_deg, el_deg):
        # Elevation clamped per your normal controller rules
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
        """
        Optional manual call if you want to persist AZ=0 to disk.
        """
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

    def _update_current_position(self):
        # Az
        az_unwrapped = self.az_tracker.read()
        az_phys = position.az_unwrapped_to_physical(az_unwrapped, self.cal["az_offset_deg"])

        # El
        el_raw = self.enc_el.read_degrees_filtered()
        el_phys = position.el_raw_to_physical(el_raw, self.cal["el_offset_deg"])

        with self._lock:
            self._cur_az_phys = az_phys % 360.0
            self._cur_el_phys = el_phys

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
                    last_arrival_target = self._last_arrival_target

                if stop_req or (target_az is None and target_el is None):
                    movement.stop_motor(self.motor_az)
                    movement.stop_motor(self.motor_el)
                else:
                    cur_az_unwrapped = self.az_tracker.unwrapped
                    cur_az_phys, cur_el_phys = self.get_position()

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

                        # EL safety
                        if (cur_el_phys <= config.EL_MIN_DEG + config.DEADBAND_DEG) and (el_err < 0):
                            movement.stop_motor(self.motor_el)
                        elif (cur_el_phys >= config.EL_MAX_DEG - config.DEADBAND_DEG) and (el_err > 0):
                            movement.stop_motor(self.motor_el)
                        else:
                            movement.drive_toward_error(self.motor_el, config.M2_FORWARD_SIGN, el_err)

                        movement.drive_toward_error(self.motor_az, config.M1_FORWARD_SIGN, az_err)

            except Exception:
                try:
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
        rc.set_target(args.az if args.az is not None else 0.0,
                      args.el if args.el is not None else 0.0)
        while True:
            time.sleep(1.0)
