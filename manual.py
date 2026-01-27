# manual.py
# Keyboard jog control for your rotator.
#
# Controls:
#   A / D  : Azimuth jog (left/right)
#   W / S  : Elevation jog (up/down)
#   Space  : Stop all motors
#   P      : Print current AZ/EL
#   H      : Save AZ home here (AZ=0)
#   J      : Save EL zero here (EL=0 horizon)
#   Q      : Quit
#
# Notes:
# - Uses your existing libraries/config (config.py, position.py, movement.py).
# - Runs in the terminal. Requires sudo for I2C/motors like your other scripts.
# - This is a *jog* tool: motors move while a key is held (key repeat works too).
# - Elevation LIMITS are DISABLED in this manual tool (no 0..90 clamping).

import sys
import time
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT

import config
import position
import movement


# ---------------------------
# Simple terminal key reading
# ---------------------------
def _getch():
    """
    Read a single keypress (non-blocking-ish by using raw mode).
    Works over SSH too.
    """
    import termios
    import tty
    import select

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    cal = config.load_cal()

    # Hardware init
    mh = movement.init_motorhat()
    motor_az = mh.getMotor(1)  # M1 = AZ
    motor_el = mh.getMotor(2)  # M2 = EL

    enc_az = position.AS5600(config.ENCODER1_BUS, name="E1")
    enc_el = position.AS5600(config.ENCODER2_BUS, name="E2")
    az_tracker = position.UnwrappedAngle(enc_az)

    # Prime stable reads (and reduce initial jitter)
    position.stable_read_unwrapped(az_tracker, seconds=0.3)
    position.stable_read_wrapped(enc_el, seconds=0.3)

    def stop_all():
        movement.stop_motor(motor_az)
        movement.stop_motor(motor_el)

    atexit.register(stop_all)

    def get_positions():
        # AZ physical is derived from unwrapped minus az_offset (wrapped 0..360)
        az_unwrapped = az_tracker.read()
        az_phys = position.az_unwrapped_to_physical(az_unwrapped, cal["az_offset_deg"]) % 360.0

        # EL physical is wrap_delta(raw, el_offset). For manual mode, we do NOT clamp.
        el_raw = enc_el.read_degrees_filtered()
        el_phys = position.el_raw_to_physical(el_raw, cal["el_offset_deg"])
        return az_phys, el_phys

    def print_positions(prefix="[POS]"):
        az, el = get_positions()
        print(
            f"{prefix} AZ={az:7.2f}°  EL={el:7.2f}°   "
            f"(az_off={cal['az_offset_deg']:.2f}  el_off={cal['el_offset_deg']:.2f})",
            flush=True
        )

    def save_az_home_here():
        # Want current physical AZ = 0 => az_offset = current raw wrapped (unwrapped % 360)
        cur_unwrapped = position.stable_read_unwrapped(az_tracker, seconds=0.25)
        cal["az_offset_deg"] = cur_unwrapped % 360.0
        config.save_cal(cal)
        print(f"[CAL] Saved AZ home here. az_offset_deg={cal['az_offset_deg']:.6f}", flush=True)

    def save_el_zero_here():
        # Want current physical EL = 0 => el_offset = current raw wrapped
        cur_raw = position.stable_read_wrapped(enc_el, seconds=0.25)
        cal["el_offset_deg"] = cur_raw % 360.0
        config.save_cal(cal)
        print(f"[CAL] Saved EL zero here. el_offset_deg={cal['el_offset_deg']:.6f}", flush=True)

    # Jog tuning
    JOG_SPEED_AZ = 120
    JOG_SPEED_EL = 120

    # Key repeat / command hold behavior
    HOLD_SEC = 0.20
    last_cmd_time = 0.0
    active_cmd = None  # one of: 'AZ_LEFT','AZ_RIGHT','EL_UP','EL_DOWN'

    print("\nManual jog control ready.\n")
    print("Controls:")
    print("  A/D : AZ jog")
    print("  W/S : EL jog")
    print("  Space: stop motors")
    print("  P : print positions")
    print("  H : save AZ home here (AZ=0)")
    print("  J : save EL zero here (EL=0)")
    print("  Q : quit\n")
    print("NOTE: Elevation limits are DISABLED in this manual tool.\n")
    print_positions(prefix="[START]")

    try:
        while True:
            k = _getch()
            now = time.time()

            if k:
                kk = k.lower()

                if kk == "q":
                    print("\n[QUIT] Exiting manual jog.", flush=True)
                    break

                if kk == " ":
                    active_cmd = None
                    stop_all()
                    print("[STOP] Motors released.", flush=True)
                    continue

                if kk == "p":
                    print_positions()
                    continue

                if kk == "h":
                    stop_all()
                    save_az_home_here()
                    print_positions(prefix="[POS]")
                    continue

                if kk == "j":
                    stop_all()
                    save_el_zero_here()
                    print_positions(prefix="[POS]")
                    continue

                if kk == "a":
                    active_cmd = "AZ_LEFT"
                    last_cmd_time = now
                elif kk == "d":
                    active_cmd = "AZ_RIGHT"
                    last_cmd_time = now
                elif kk == "w":
                    active_cmd = "EL_UP"
                    last_cmd_time = now
                elif kk == "s":
                    active_cmd = "EL_DOWN"
                    last_cmd_time = now

            # If no recent command, release motors
            if active_cmd is None or (now - last_cmd_time) > HOLD_SEC:
                active_cmd = None
                stop_all()
                continue

            # Apply the active jog command
            if active_cmd.startswith("AZ"):
                # Jog AZ: drive continuously in desired direction
                if active_cmd == "AZ_RIGHT":
                    err = +999.0  # want increase
                else:
                    err = -999.0  # want decrease

                movement.motor_set_speed(motor_az, JOG_SPEED_AZ)
                direction = movement.motor_dir_for_error(config.M1_FORWARD_SIGN, err)
                motor_az.run(direction)

                # Keep EL released unless it's being driven
                movement.stop_motor(motor_el)

            elif active_cmd.startswith("EL"):
                # Jog EL: NO limit checks in manual mode
                movement.stop_motor(motor_az)

                if active_cmd == "EL_UP":
                    err = +999.0
                else:
                    err = -999.0

                movement.motor_set_speed(motor_el, JOG_SPEED_EL)
                direction = movement.motor_dir_for_error(config.M2_FORWARD_SIGN, err)
                motor_el.run(direction)

    finally:
        stop_all()


if __name__ == "__main__":
    main()
