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

FAST_SPEED  = 150
SLOW_SPEED  = 95
CREEP_SPEED = 70

SLOW_WINDOW_DEG  = 10.0
CREEP_WINDOW_DEG = 2.5
DEADBAND_DEG     = 0.6

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
