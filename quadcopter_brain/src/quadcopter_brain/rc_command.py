import numpy

from apm_mode import APMMode


class RCCommand(object):
    def __init__(self, parameters={}):
        self._NO_CHANGE = 65535

        self._roll_min_pwm = 1100
        self._roll_max_pwm = 1900
        self._roll = self.compute_pwm(
            parameters.get("roll", 0.5),
            self._roll_min_pwm,
            self._roll_max_pwm)

        self._pitch_min_pwm = 1900
        self._pitch_max_pwm = 1100
        self._pitch = self.compute_pwm(
            parameters.get("pitch", 0.5),
            self._pitch_min_pwm,
            self._pitch_max_pwm)

        self._yaw_min_pwm = 1100
        self._yaw_max_pwm = 1900
        self._yaw = self.compute_pwm(
            parameters.get("yaw", 0.5),
            self._yaw_min_pwm,
            self._yaw_max_pwm)

        self._throttle_min_pwm = 1100
        self._throttle_max_pwm = 1900
        self._throttle = self.compute_pwm(
            parameters.get("throttle", 0.5),
            self._throttle_min_pwm,
            self._throttle_max_pwm)

        self._apm_mode = APMMode.LOITER
        self._gimbal_yaw = self._NO_CHANGE
        self._gimbal_tilt = self._NO_CHANGE
        self._gimbal_roll = self._NO_CHANGE

    def compute_pwm(self, percent, minimum, maximum):
        if (percent < 0.0) or (percent > 1.0):
            raise ValueError

        return int(numpy.interp(percent, [0, 1.0], [minimum, maximum]))

    def to_roscopter(self):
        return [self._roll, self._pitch, self._throttle, 
                self._yaw, self._apm_mode, self._gimbal_yaw,
                self._gimbal_tilt, self._gimbal_roll]
