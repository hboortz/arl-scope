import numpy


class RCCommand(object):
    def __init__(self):
        self._roll = 1500
        self._roll_min_pwm = 1100
        self._roll_max_pwm = 1900
        
        self._pitch = 1500
        self._pitch_min_pwm = 1100
        self._pitch_max_pwm = 1900
        
        self._yaw = 1500
        self._yaw_min_pwm = 1100
        self._yaw_max_pwm = 1900

        self._throttle = 1500
        self._throttle_min_pwm = 1100
        self._throttle_max_pwm = 1900
        
        self._apm_mode = 1146
        self._gimbal_yaw = 1000
        self._gimbal_tilt = 1000
        self._gimbal_roll = 1000

    def compute_pwm_signal(self, percent, minimum, maximum):
        if (percent < 0.0) or (percent > 1.0):
            raise ValueError

        return numpy.interp(percent, [0, 1.0], [minimum, maximum])

    def set_roll(self, percent):
        self._roll = self.compute_pwm_signal(
            percent, self._roll_min_pwm, self._roll_max_pwm)

    def set_pitch(self, percent):
        self._pitch = self.compute_pwm_signal(
            percent, self._pitch_min_pwm, self._pitch_max_pwm)
    
    def set_throttle(self, percent):
        self._throttle = self.compute_pwm_signal(
            percent, self._throttle_min_pwm, self._throttle_max_pwm)

    def set_yaw(self, percent):
        self._yaw = self.compute_pwm_signal(
            percent, self._yaw_min_pwm, self._yaw_max_pwm)

    def to_roscopter(self):
        print "[%d, %d, %d, %d, %d, %d, %d, %d]" % (
            self._roll, self._pitch, self._yaw,
            self._throttle, self._apm_mode, self._gimbal_yaw,
            self._gimbal_tilt, self._gimbal_roll)
        blah = "[%d, %d, %d, %d, %d, %d, %d, %d]" % (
            self._roll, self._pitch, self._yaw,
            self._throttle, self._apm_mode, self._gimbal_yaw,
            self._gimbal_tilt, self._gimbal_roll)
        return str(blah)
