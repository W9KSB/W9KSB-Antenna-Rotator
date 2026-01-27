# position.py
import time
import statistics
import Adafruit_GPIO.I2C as I2C

import config


def wrap_delta_deg(a, b):
    """
    Smallest signed delta (a - b) in (-180, 180]
    """
    return (a - b + 180.0) % 360.0 - 180.0


class AS5600:
    RAW_ANGLE_MSB = 0x0E
    RAW_ANGLE_LSB = 0x0F

    def __init__(self, busnum, name="ENC"):
        self.i2c = I2C.get_i2c_device(config.AS5600_ADDR, busnum=busnum)
        self.name = name
        self.last_deg = None

    def _read_raw_once(self):
        high = self.i2c.readU8(self.RAW_ANGLE_MSB)
        low  = self.i2c.readU8(self.RAW_ANGLE_LSB)
        return ((high << 8) | low) & 0x0FFF

    def read_degrees_once(self):
        raw = self._read_raw_once()
        return (raw * 360.0) / 4096.0

    def read_degrees_filtered(self):
        samples = []
        for _ in range(config.SAMPLES_PER_READ):
            for attempt in range(config.I2C_RETRIES):
                try:
                    samples.append(self.read_degrees_once())
                    break
                except Exception:
                    if attempt == config.I2C_RETRIES - 1:
                        if self.last_deg is not None:
                            return self.last_deg
                        raise
                    time.sleep(0.005)
            time.sleep(0.002)

        med = statistics.median(samples)

        if self.last_deg is not None:
            jump = abs(wrap_delta_deg(med, self.last_deg))
            if jump > config.MAX_JUMP_DEG:
                med = self.last_deg

        self.last_deg = med
        return med


class UnwrappedAngle:
    """
    Tracks continuous degrees by accumulating wrap-safe deltas
    from the filtered wrapped reading.
    """
    def __init__(self, encoder: AS5600):
        self.enc = encoder
        self.initialized = False
        self.last_wrapped = None
        self.unwrapped = 0.0

    def read(self):
        w = self.enc.read_degrees_filtered()
        if not self.initialized:
            self.initialized = True
            self.last_wrapped = w
            self.unwrapped = w
            return self.unwrapped

        d = wrap_delta_deg(w, self.last_wrapped)
        self.unwrapped += d
        self.last_wrapped = w
        return self.unwrapped


def stable_read_wrapped(enc: AS5600, seconds=0.25):
    vals = []
    end = time.time() + seconds
    while time.time() < end:
        vals.append(enc.read_degrees_filtered())
        time.sleep(0.02)
    return statistics.median(vals) if vals else enc.read_degrees_filtered()


def stable_read_unwrapped(tracker: UnwrappedAngle, seconds=0.25):
    vals = []
    end = time.time() + seconds
    while time.time() < end:
        vals.append(tracker.read())
        time.sleep(0.02)
    return statistics.median(vals) if vals else tracker.read()


# ----------------------------
# Elevation mapping (raw <-> physical)
# ----------------------------

def el_raw_to_physical(raw_deg, el_offset_deg):
    """
    Convert raw encoder (0..360) to physical elevation degrees using offset.
    Returns a signed value near the real elevation (typically 0..90).
    """
    return wrap_delta_deg(raw_deg, el_offset_deg)


# ----------------------------
# Azimuth mapping (unwrapped/raw <-> physical)
# ----------------------------

def az_unwrapped_to_physical(unwrapped_deg, az_offset_deg):
    """
    Physical az is (raw - offset) wrapped to 0..360
    """
    return (unwrapped_deg - az_offset_deg) % 360.0


def nearest_unwrapped_target(current_unwrapped, target_az_phys_deg, az_offset_deg):
    """
    Given a desired physical azimuth (0..360 reference),
    convert it to a raw wrapped target using az_offset, then pick the nearest
    equivalent unwrapped value (shortest path).
    """
    t_phys = target_az_phys_deg % 360.0
    raw_target_wrapped = (az_offset_deg + t_phys) % 360.0
    k = round((current_unwrapped - raw_target_wrapped) / 360.0)
    return raw_target_wrapped + 360.0 * k
