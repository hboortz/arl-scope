import unittest

import rc_command


class TestRCCommand(unittest.TestCase):
    def setUp(self):
        self.rc_command = rc_command.RCCommand()

    def test_to_mavlink(self):
        mavlink_command = self.rc_command.to_mavlink()
        true_command = "command: [1500, 1500, 1500, 1500, 1146, -1, -1, -1]"
        self.assertEqual(mavlink_command, true_command)    


if __name__ == '__main__':
    unittest.main()
