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
        out = self.keys('DADAWW')
        self.assertEqual(out[-2:], [(0, 0), (0, 0)])           # code keys W W didn't drive
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'] + 1)
        self.assertIn('*', self.app.version)

    def test_codes_for_range_speed_and_save(self):
        self.keys('DADAAW'); clock[0] += 100
        self.keys('DADADS'); clock[0] += 100
        self.keys('DADASS')
        self.assertEqual(self.app.cal['travel'], self.ns['STEER_TRAVEL'] + 1)
        self.assertEqual(self.app.cal['speed'], self.ns['SPEED'] - 10)
        self.assertNotIn('*', self.app.version)
        again = self.ns['TankApp']()                          # reboot: saved values load
        self.assertEqual(again.cal, self.app.cal)
        self.assertEqual(again.throttles[b'W'][0], abs(self.fwd[0]) // self.fwd[0] * (self.ns['SPEED'] - 10))

    def test_revert(self):
        self.keys('DADAWW'); clock[0] += 100
        self.keys('DADASA')
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'])

    def test_slow_keys_are_not_a_code(self):
        self.keys('DADAWW', gap=100)                          # remote driving pace
        self.assertEqual(self.app.cal['center'], self.ns['STEER_CENTER'])

    def test_limits(self):
        for _ in range(200):
            self.keys('DADAAW'); clock[0] += 100
        c = self.app.cal
        self.assertLessEqual(c['center'] + c['travel'], 180)
        self.assertGreaterEqual(c['center'] - c['travel'], 0)

    def test_version_fits_the_service(self):
        self.assertLessEqual(len(self.app.version + '*'), 48)


if __name__ == '__main__':
    unittest.main()
