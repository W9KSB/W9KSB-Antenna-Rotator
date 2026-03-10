# config.py
import json
import os

# ----------------------------
# Hardware mapping
# ----------------------------
AS5600_ADDR = 0x36

ENCODER1_BUS = 1   # Encoder1 -> Motor1 (Az)
ENCODER2_BUS = 3   # Encoder2 -> Motor2 (El)

MOTOR_HAT_ADDR = 0x60
MOTOR_HAT_BUS = 1

# Direction mapping from your tests:
# +1 means: commanding FORWARD causes encoder angle to increase
M1_FORWARD_SIGN = -1   # Motor1 FORWARD decreases encoder
M2_FORWARD_SIGN = +1   # Motor2 FORWARD increases encoder

# ----------------------------
# Elevation constraints (physical)
# ----------------------------
EL_MIN_DEG = 0.0
EL_MAX_DEG = 90.0

# ----------------------------
# Control tuning
# ----------------------------
CONTROL_HZ = 15.0                  # controller loop rate
READ_INTERVAL = 0.05               # used for debug-style loops (not required by control loop)

SAMPLES_PER_READ = 5
I2C_RETRIES = 3
MAX_JUMP_DEG = 25.0

# These are used by movement.py for AZ and as general fallbacks.
FAST_SPEED  = 170
SLOW_SPEED  = 125
CREEP_SPEED = 100

SLOW_WINDOW_DEG  = 10.0
CREEP_WINDOW_DEG = 2.5
DEADBAND_DEG     = 1

# ----------------------------
# Elevation UP travel schedule (used when not in approach/creep)
# ----------------------------
# Based on *current* elevation while moving UP
EL_UP_SPEED_0_25   = 250
EL_UP_SPEED_25_45  = 200
EL_UP_SPEED_45_MAX = 125

EL_UP_BREAK_1_DEG = 25.0
EL_UP_BREAK_2_DEG = 45.0

# ----------------------------
# Elevation DOWN travel speed (used when not in approach/creep)
# ----------------------------
# After breakaway is achieved, and while not near the target,
# DOWN uses this steady "travel" speed.
EL_DOWN_TRAVEL_SPEED = 200

# ----------------------------
# Elevation breakaway search + stall recovery (NEW controller behavior)
# ----------------------------
# Movement detection (encoder raw degrees). Must be above noise.
EL_MOVE_THRESH_DEG = 0.25
EL_MOVE_WINDOW_S   = 0.30

# If we are commanding motion but we do not detect movement long enough,
# we consider it stalled and re-enter breakaway search.
EL_STALL_TIME_S = 0.45

# Breakaway search (UP)
EL_UP_BREAKAWAY_START_SPEED    = 50
EL_UP_BREAKAWAY_STEP_SPEED     = 50
EL_UP_BREAKAWAY_MAX_SPEED      = 255
EL_UP_BREAKAWAY_INTERVAL_S     = 0.30

# Breakaway search (DOWN)
# Your testing showed ~200-210 breaks free at ~80° EL, so keep max flexible.
EL_DOWN_BREAKAWAY_START_SPEED  = 100
EL_DOWN_BREAKAWAY_STEP_SPEED   = 50
EL_DOWN_BREAKAWAY_MAX_SPEED    = 255
EL_DOWN_BREAKAWAY_INTERVAL_S   = 0.30

# ----------------------------
# Elevation approach / creep (smooth near target, but will re-kick if it stalls)
# ----------------------------
EL_APPROACH_WINDOW_DEG = 8.0
EL_CREEP_WINDOW_DEG    = 2.5

EL_APPROACH_SPEED      = 100
EL_CREEP_SPEED         = 60

# Optional: settle time when changing EL direction
EL_DIR_CHANGE_SETTLE_S = 0.25

# ----------------------------
# (Removed) old "max power zones" + governor settings
# ----------------------------
# These are NOT used by the new controller.py you just installed:
#   EL_UP_MAXZONE_*
#   EL_DOWN_MAXZONE_*
#   EL_DOWN_GOV_*
# You can re-add later if you decide to bring that behavior back.

# ----------------------------
# Calibration storage
# ----------------------------
CAL_FILE = "rotator_cal.json"


def load_cal():
    """
    Loads offsets from rotator_cal.json if present.
    Offsets are RAW wrapped degrees (0..360) that correspond to physical zero.
      az_offset_deg: raw E1 angle that equals AZ=0
      el_offset_deg: raw E2 angle that equals EL=0 (horizon)
    """
    cal = {
        "az_offset_deg": 0.0,
        "el_offset_deg": 0.0,
    }
    if os.path.exists(CAL_FILE):
        try:
            with open(CAL_FILE, "r") as f:
                cal.update(json.load(f))
        except Exception:
            pass
    return cal


def save_cal(cal):
    with open(CAL_FILE, "w") as f:
        json.dump(cal, f, indent=2, sort_keys=True)
