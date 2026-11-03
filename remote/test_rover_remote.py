import unittest

import rover_remote as r

MID_Y, MID_X, T = r.MID[r.PIN_LY], r.MID[r.PIN_RX], r.THRESHOLD


class KeysFor(unittest.TestCase):
    def test_centred_sticks_send_nothing(self):
        self.assertEqual(r.keys_for(MID_Y, MID_X), '')
        self.assertEqual(r.keys_for(MID_Y + T - 1, MID_X - T + 1), '')

    def test_throttle_and_steer(self):
        self.assertEqual(r.keys_for(MID_Y + T, MID_X), 'W')
        self.assertEqual(r.keys_for(MID_Y - T, MID_X), 'S')
        self.assertEqual(r.keys_for(MID_Y, 18), 'A')            # right stick left measured ~18
        self.assertEqual(r.keys_for(MID_Y, 4086), 'D')
        self.assertEqual(r.keys_for(3159, 18), 'WA')            # left stick up measured ~3159


class Boot(unittest.TestCase):
    def test_switch_middle(self):
        self.assertTrue(r.switch_centred(1811))
        self.assertFalse(r.switch_centred(0))


if __name__ == '__main__':
    unittest.main()
