import unittest

import rover_remote as r

MID_Y, MID_X, T = r.MID[r.PIN_LY], r.MID[r.PIN_RX], r.THRESHOLD


class KeysFor(unittest.TestCase):
    def test_centred_sticks_send_nothing(self):
        self.assertEqual(r.keys_for(MID_Y, MID_X), '')
        self.assertEqual(r.keys_for(MID_Y + T - 1, MID_X - T + 1), '')

    def test_throttle(self):
        self.assertEqual(r.keys_for(MID_Y + T, MID_X), 'W')
        self.assertEqual(r.keys_for(MID_Y - T, MID_X), 'S')

    def test_steer_alone(self):
        self.assertEqual(r.keys_for(MID_Y, MID_X - T), 'A')
        self.assertEqual(r.keys_for(MID_Y, MID_X + T), 'D')

    def test_drive_and_steer_together(self):
        self.assertEqual(r.keys_for(MID_Y + T, MID_X - T), 'WA')
        self.assertEqual(r.keys_for(MID_Y - T, MID_X + T), 'SD')

    def test_boot_switch(self):
        self.assertTrue(r.switch_centred(1811))   # rest reading on this remote
        self.assertFalse(r.switch_centred(3900))
        self.assertFalse(r.switch_centred(300))


if __name__ == '__main__':
    unittest.main()
