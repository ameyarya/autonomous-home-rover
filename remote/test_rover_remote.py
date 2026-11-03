import unittest

import rover_remote as r

MID_Y, MID_X, T = r.MID[r.PIN_LY], r.MID[r.PIN_RX], r.THRESHOLD
LX, LEVER = r.MID[r.PIN_LX], r.MID[r.PIN_LEVER]


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


class Modes(unittest.TestCase):
    def test_switch_positions(self):                           # measured 0 / ~1800 / 4076
        self.assertEqual(r.mode_for(0), r.STEER_MODE)
        self.assertEqual(r.mode_for(1800), r.DRIVE_MODE)
        self.assertEqual(r.mode_for(4076), r.SPEED_MODE)
        self.assertTrue(r.switch_centred(1811))

    def test_flick_steps_once_then_repeats(self):
        f = r.Flick()
        self.assertEqual(f.step(1, 0), 1)
        self.assertEqual(f.step(1, 100), 0)
        self.assertEqual(f.step(1, 600), 1)                     # held: repeat
        self.assertEqual(f.step(1, 700), 0)
        self.assertEqual(f.step(1, 900), 1)
        self.assertEqual(f.step(0, 950), 0)
        self.assertEqual(f.step(-1, 1000), -1)

    def test_edits_per_mode(self):
        c, l = r.Flick(), r.Flick()
        self.assertEqual(r.edits_for(r.STEER_MODE, 4000, 4077, c, l, 0), [('travel', 1), ('center', 1)])
        c, l = r.Flick(), r.Flick()
        self.assertEqual(r.edits_for(r.SPEED_MODE, 4000, 0, c, l, 0), [('speed', -1)])   # left stick ignored
        c, l = r.Flick(), r.Flick()
        self.assertEqual(r.edits_for(r.DRIVE_MODE, LX, LEVER, c, l, 0), [])

    def test_codes_match_the_rover_app(self):
        import re
        from pathlib import Path
        app = (Path(__file__).resolve().parent.parent / 'drive/rover_drive_app.py').read_text()
        table = {code: (what, int(step)) for code, what, step in re.findall(r"b'(\w\w)': \('(\w+)', (-?\d+)\)", app)}
        for edit, code in r.CODES.items():
            if edit == 'save':
                self.assertEqual(table[code][0], 'save')
            else:
                self.assertEqual((table[code][0], table[code][1] > 0), (edit[0], edit[1] > 0), code)


if __name__ == '__main__':
    unittest.main()
