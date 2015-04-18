import unittest

import rc_command


class TestRCCommand(unittest.TestCase):
    def test_to_roscopter(self):
        command = rc_command.RCCommand()
        roscopter_command = command.to_roscopter()
        true_command = [1500, 1500, 1500, 1500, 1581, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        command = rc_command.RCCommand(
            {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0})
        roscopter_command = command.to_roscopter()
        true_command = [1100, 1100, 1100, 1100, 1581, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        command = rc_command.RCCommand(
            {"roll": 1.0, "pitch": 1.0, "yaw": 1.0, "throttle": 1.0})
        roscopter_command = command.to_roscopter()
        true_command = [1900, 1900, 1900, 1900, 1581, 65535, 65535, 65535]
        self.assertEqual(roscopter_command, true_command)

        for pwm in roscopter_command:
            self.assertEqual(type(pwm), int)

    def test_compute_pwm(self):
        command = rc_command.RCCommand()

        self.assertRaises(
            ValueError, command.compute_pwm, 2.0, 1100, 1900)
        self.assertRaises(
            ValueError, command.compute_pwm, -1.0, 1100, 1900)

        pwm_signal = command.compute_pwm(0.0, 1100, 1900)
        self.assertEqual(pwm_signal, 1100)
        pwm_signal = command.compute_pwm(0.5, 1100, 1900)
        self.assertEqual(pwm_signal, 1500)
        pwm_signal = command.compute_pwm(1.0, 1100, 1900)
        self.assertEqual(pwm_signal, 1900)


if __name__ == '__main__':
    unittest.main()
