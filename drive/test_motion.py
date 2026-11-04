"""Run: cd drive && python3 -m unittest test_motion"""
import math
import tempfile
import unittest
from pathlib import Path

import drive_server
import motion
from test_drive_server import FakeTx

M = {'forward_cm_s': 40.0, 'reverse_cm_s': 30.0, 'turn_radius_cm': 50.0, 'power': 1200}


class Plan(unittest.TestCase):
    def test_straight_moves(self):
        self.assertEqual(motion.plan('forward', 100, M, 1200), ('W', 2.5))
        self.assertEqual(motion.plan('back', 30, M, 1200), ('S', 1.0))

    def test_turn_is_a_full_lock_arc(self):
        key, s = motion.plan('left', 90, M, 1200)
        self.assertEqual(key, 'A')
        self.assertAlmostEqual(s, 50 * math.pi / 2 / 40)
        self.assertEqual(motion.plan('right', 45, M, 1200)[0], 'D')

    def test_speed_scales_with_power(self):
        self.assertAlmostEqual(motion.plan('forward', 100, M, 600)[1], 5.0)

    def test_needs_measurements_and_limits_length(self):
        with self.assertRaises(ValueError):
            motion.plan('forward', 10, {k: None for k in motion.FIELDS}, 1200)
        with self.assertRaises(ValueError):
            motion.plan('forward', 1000, M, 1200)                   # 25 s > the limit


class Radius(unittest.TestCase):
    def test_radius_from_a_short_arc(self):
        self.assertAlmostEqual(motion.radius_from_arc(50, 50), 50)      # quarter circle
        self.assertAlmostEqual(motion.radius_from_arc(40, 20), 50)
        with self.assertRaises(ValueError):
            motion.radius_from_arc(40, 0)


class Odometer(unittest.TestCase):
    def test_straight_then_quarter_turn(self):
        o = motion.Odometer()
        o.add('W', 1.0, M, 1200)
        self.assertEqual(o.as_dict(), {'distance_cm': 40.0, 'x_cm': 40.0, 'y_cm': 0.0, 'heading_deg': 0.0})
        o.add('A', 50 * math.pi / 2 / 40, M, 1200)                  # 90 deg left arc
        d = o.as_dict()
        self.assertAlmostEqual(d['heading_deg'], 90.0, places=0)
        self.assertAlmostEqual(d['x_cm'], 90.0, delta=1)
        self.assertAlmostEqual(d['y_cm'], 50.0, delta=1)

    def test_reverse_and_steer_only(self):
        o = motion.Odometer()
        o.add('S', 1.0, M, 1200)
        o.add('L', 5.0, M, 1200)                                     # wheels only: no travel
        self.assertEqual(o.as_dict()['x_cm'], -30.0)
        self.assertEqual(o.as_dict()['distance_cm'], 30.0)


class Moves(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.old = motion.MOTION_FILE
        motion.MOTION_FILE = Path(self.tmp.name) / 'motion.json'
        self.tx = FakeTx()
        self.s = drive_server.DriveState(self.tx)
        self.s.installed = 'rover-drive-0.8+cabc123 c90 t40 s1200|0'
        self.s.save_motion(M)

    def tearDown(self):
        motion.MOTION_FILE = self.old
        self.tmp.cleanup()

    def test_move_needs_arm(self):
        with self.assertRaises(PermissionError):
            self.s.move('forward', 10)
        self.assertEqual(self.tx.calls, [])

    def test_forward_move_drives_for_the_planned_time_and_counts(self):
        self.s.arm(True)
        r = self.s.move('forward', 20)                               # 0.5 s at 40 cm/s
        self.assertGreaterEqual(self.tx.calls.count('W'), 3)
        self.assertEqual(self.tx.calls[-1], 'X')
        self.assertAlmostEqual(r['ms'], 500, delta=30)
        self.assertAlmostEqual(r['odometer']['distance_cm'], 20, delta=1)

    def test_test_run_and_saved_values(self):
        self.s.arm(True)
        r = self.s.move('test_reverse')
        self.assertAlmostEqual(r['ms'], motion.TEST_S['reverse'] * 1000, delta=40)
        self.assertEqual(motion.load()['turn_radius_cm'], 50.0)

    def test_bad_amount(self):
        self.s.arm(True)
        with self.assertRaises(ValueError):
            self.s.move('left', -5)


if __name__ == '__main__':
    unittest.main()
