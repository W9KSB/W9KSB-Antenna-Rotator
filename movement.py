# movement.py
from Adafruit_MotorHAT import Adafruit_MotorHAT
import Adafruit_GPIO.I2C as I2C

import config


class FixedBusI2C:
    """
    MotorHAT library workaround: provide get_i2c_device() and force busnum.
    """
    def __init__(self, busnum):
        self.busnum = busnum

    def get_i2c_device(self, address, **kwargs):
        return I2C.get_i2c_device(address, busnum=self.busnum, **kwargs)


def init_motorhat():
    mh_i2c = FixedBusI2C(config.MOTOR_HAT_BUS)
    mh = Adafruit_MotorHAT(addr=config.MOTOR_HAT_ADDR, i2c=mh_i2c)
    return mh


def motor_set_speed(motor, speed):
    motor.setSpeed(max(0, min(255, int(speed))))


def motor_dir_for_error(forward_sign, err_deg):
    """
    err_deg > 0 means we want encoder angle to increase.
    forward_sign says whether FORWARD increases (+1) or decreases (-1).
    """
    want_increase = err_deg > 0
    if want_increase:
        return Adafruit_MotorHAT.FORWARD if forward_sign == +1 else Adafruit_MotorHAT.BACKWARD
    else:
        return Adafruit_MotorHAT.BACKWARD if forward_sign == +1 else Adafruit_MotorHAT.FORWARD


def speed_for_error(abs_err):
    if abs_err <= config.CREEP_WINDOW_DEG:
        return config.CREEP_SPEED
    if abs_err <= config.SLOW_WINDOW_DEG:
        return config.SLOW_SPEED
    return config.FAST_SPEED


def drive_toward_error(motor, forward_sign, err_deg):
    """
    Non-blocking: for this tick, drive motor toward reducing err_deg.
    Returns True if "at target" (within deadband), else False.
    """
    if abs(err_deg) <= config.DEADBAND_DEG:
        motor.run(Adafruit_MotorHAT.RELEASE)
        return True

    spd = speed_for_error(abs(err_deg))
    direction = motor_dir_for_error(forward_sign, err_deg)
    motor_set_speed(motor, spd)
    motor.run(direction)
    return False


def stop_motor(motor):
    motor.run(Adafruit_MotorHAT.RELEASE)
