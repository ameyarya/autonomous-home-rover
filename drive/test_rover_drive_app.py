"""Run: cd drive && python3 -m unittest test_rover_drive_app
The receiver app on CPython with a fake MicroPython clock (no servo: it isn't the active app)."""
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

clock = [0]
fake_time = types.ModuleType('time')
fake_time.ticks_ms = lambda: clock[0]
fake_time.ticks_diff = lambda a, b: a - b
fake_time.ticks_add = lambda a, b: a + b
SOURCE = (Path(__file__).resolve().parent / 'rover_drive_app.py').read_text()


def load():
    real = sys.modules['time']
    sys.modules['time'] = fake_time
    try:
        ns = {'__name__': 'tank_motor_app'}
        exec(SOURCE, ns)
        return ns
    finally:
        sys.modules['time'] = real


class AppTest(unittest.TestCase):
    def setUp(self):
        self.cwd = os.getcwd()
        self.tmp = tempfile.TemporaryDirectory()
        os.chdir(self.tmp.name)                 # CAL_FILE lands here
        clock[0] = 1000
        self.ns = load()
        self.app = self.ns['TankApp']()
        self.fwd = self.app.throttles[b'W']

    def tearDown(self):
        os.chdir(self.cwd)
        self.tmp.cleanup()

    def keys(self, keys, gap=5):
        out = []
        for k in keys:
            out.append(self.app.command(k.encode()))
            clock[0] += gap
        return out

    def test_ww_a_steers_gently(self):
        self.keys('WWA', gap=5)
        self.assertAlmostEqual(self.app.wanted, -self.ns['GENTLE'])
        self.keys('WA', gap=5)                                  # plain W A: full lock again
        self.assertEqual(self.app.wanted, -1)

    def test_ww_centres_now(self):
        self.keys('WA', gap=5)
        clock[0] += 100
        self.app.command(b'W')                                  # a lone W keeps the recent steer
        self.assertTrue(self.app.fresh(self.app.wanted_at, clock[0]))
        clock[0] += 5
        self.app.command(b'W')                                  # W W: centred
        self.assertFalse(self.app.fresh(self.app.wanted_at, clock[0]))

    def test_remote_pace_never_matches_the_patterns(self):
        self.keys('WWA', gap=50)                                # the handheld remote: 50+ ms apart
        self.assertEqual(self.app.wanted, -1)

    def test_reverse_gentle(self):
        self.keys('SSD', gap=5)
        self.assertAlmostEqual(self.app.wanted, self.ns['GENTLE'])

    def test_service_validation_sequence(self):
        self.assertEqual(self.keys('WASDX')[-1], (0, 0))

    def test_steer_alone_does_not_drive(self):
        self.assertEqual(self.app.command(b'A'), (0, 0))

    def test_drive_and_steer_combine(self):
        self.app.command(b'W'); clock[0] += 100
        self.assertEqual(self.app.command(b'A'), self.fwd)
        clock[0] += 450
        self.assertEqual(self.app.command(b'D'), (0, 0))      # throttle went stale

    def test_code_changes_centre_without_driving(self):
        out = self.keys('XXXXAAA')
        self.assertEqual(out[-3:], [(0, 0)] * 3)               # code keys never drive
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'] + 1)
        self.assertIn('*', self.app.version)

    def test_codes_for_range_speed_and_save(self):
        self.keys('XXXXADA'); clock[0] += 100
        self.keys('XXXXDAD'); clock[0] += 100
        self.keys('XXXXDDA')
        self.assertEqual(self.app.cal['travel'], self.ns['STEER_TRAVEL'] + 1)
        self.assertEqual(self.app.cal['speed'], self.ns['SPEED'] - 10)
        self.assertNotIn('*', self.app.version)
        again = self.ns['TankApp']()                          # reboot: saved values load
        self.assertEqual(again.cal, self.app.cal)
        self.assertEqual(again.throttles[b'W'][0], abs(self.fwd[0]) // self.fwd[0] * (self.ns['SPEED'] - 10))

    def test_revert(self):
        self.keys('XXXXAAA'); clock[0] += 100
        self.keys('XXXXDDD')
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'])

    def test_slow_keys_are_not_a_code(self):
        self.keys('XXXXAAA', gap=120)                          # stop retries / remote pace
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'])

    def test_limits(self):
        for _ in range(200):
            self.keys('XXXXADA'); clock[0] += 100
        c = self.app.cal
        self.assertLessEqual(c['center'] + c['travel'], 180)
        self.assertGreaterEqual(c['center'] - c['travel'], 0)

    def test_missed_code_never_drives(self):
        self.app.command(b'W')
        out = self.keys('XXAAA')                               # prefix cut short: plain steering
        self.assertEqual(out[-3:], [(0, 0)] * 3)
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'])

    def test_version_fits_the_service(self):
        self.assertLessEqual(len(self.app.version + '*'), 48)


if __name__ == '__main__':
    unittest.main()
