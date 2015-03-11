import unittest

import rc_command


class TestRCCommand(unittest.TestCase):
    def setUp(self):
        self.rc_command = rc_command.RCCommand()

    def test_to_roscopter(self):
        roscopter_command = self.rc_command.to_roscopter()
        true_command = "command: [1500, 1500, 1500, 1500, 1146, -1, -1, -1]"
        self.assertEqual(roscopter_command, true_command)

    def test_compute_pwm_signal(self):
        self.assertRaises(
            ValueError, self.rc_command.compute_pwm_signal, 2.0, 1100, 1900)
        self.assertRaises(
            ValueError, self.rc_command.compute_pwm_signal, -1.0, 1100, 1900)

        pwm_signal = self.rc_command.compute_pwm_signal(0.0, 1100, 1900)
        self.assertEqual(pwm_signal, 1100)
        pwm_signal = self.rc_command.compute_pwm_signal(0.5, 1100, 1900)
        self.assertEqual(pwm_signal, 1500)
        pwm_signal = self.rc_command.compute_pwm_signal(1.0, 1100, 1900)
        self.assertEqual(pwm_signal, 1900)

    def test_set_throttle(self):
        self.rc_command.set_throttle(0.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1500, 1500, 1100, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_throttle(1.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1500, 1500, 1900, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

    def test_set_roll(self):
        self.rc_command.set_roll(1.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1900, 1500, 1500, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_roll(0.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1100, 1500, 1500, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

    def test_set_pitch(self):
        self.rc_command.set_pitch(1.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1900, 1500, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_pitch(0.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1100, 1500, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

    def test_set_yaw(self):
        self.rc_command.set_yaw(1.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1500, 1900, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_yaw(0.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = [1500, 1500, 1100, 1500, 1146, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)




if __name__ == '__main__':
    unittest.main()
