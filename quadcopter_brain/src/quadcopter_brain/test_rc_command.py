import unittest

import rc_command


class TestRCCommand(unittest.TestCase):
    def setUp(self):
        self.rc_command = rc_command.RCCommand()

    def test_to_roscopter(self):
        roscopter_command = self.rc_command.to_roscopter()
        true_command = "command: [1500, 1500, 1500, 1500, 1146, -1, -1, -1]"
        self.assertEqual(roscopter_command, true_command)

    def test_set_throttle(self):
        self.assertRaises(ValueError, self.rc_command.set_throttle, 2.0)
        self.assertRaises(ValueError, self.rc_command.set_throttle, -1.0)

        self.rc_command.set_throttle(1.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = "command: [1500, 1500, 1500, 1900, 1146, -1, -1, -1]"
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_throttle(0.5)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = "command: [1500, 1500, 1500, 1500, 1146, -1, -1, -1]"
        self.assertEqual(roscopter_command, true_command)

        self.rc_command.set_throttle(0.0)
        roscopter_command = self.rc_command.to_roscopter()
        true_command = "command: [1500, 1500, 1500, 1100, 1146, -1, -1, -1]"
        self.assertEqual(roscopter_command, true_command)


if __name__ == '__main__':
    unittest.main()
