class RCCommand(object):
    def __init__(self):
        self._roll = 1500
        self._pitch = 1500
        self._yaw = 1500
        self._throttle = 1500
        self._apm_mode = 1146
        self._gimbal_yaw = -1
        self._gimbal_tilt = -1
        self._gimbal_roll = -1

    def to_mavlink(self):
        return "command: [%d, %d, %d, %d, %d, %d, %d, %d]" % (
            self._roll, self._pitch, self._yaw, 
            self._throttle, self._apm_mode, self._gimbal_yaw, 
            self._gimbal_tilt, self._gimbal_roll)
