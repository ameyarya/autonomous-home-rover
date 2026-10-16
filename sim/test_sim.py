"""Checks of the simulator itself. Run: sim/.venv/bin/python -m unittest discover -s sim -p 'test_*.py'"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rover_sim as rs  # noqa: E402

FAR = (-5, -5)   # dog parked out of the way


def world(rover=(1.0, 2.0, 0.0), dog=None, render=False):
    w = rs.World(seed=3, render=render)
    w.place(rover, dog or rs.Dog(*FAR, 0, [('stand', 1e9, False)]))
    return w


class KinematicsTest(unittest.TestCase):
    def test_forward_pulse_goes_straight(self):
        w = world()
        w.pulse('W', 300)
        self.assertAlmostEqual(w.yaw, 0.0, places=6)
        self.assertGreater(w.x - 1.0, 0.06)
        self.assertLess(w.x - 1.0, 0.12)

    def test_a_arcs_left_d_arcs_right(self):
        left, right = world(), world()
        left.pulse('A', 300)
        right.pulse('D', 300)
        self.assertGreater(left.yaw, 0.1)
        self.assertLess(right.yaw, -0.1)
        self.assertGreater(left.y, 2.0)

    def test_reverse_is_straight_back(self):
        w = world()
        w.pulse('S', 250)
        self.assertLess(w.x, 1.0)
        self.assertAlmostEqual(w.yaw, 0.0, places=6)

    def test_wall_stops_rover(self):
        w = world(rover=(4.7, 2.0, math.pi / 2 - 0.0))
        w.place((1.0, 3.6, math.pi / 2), w.dog)        # facing the north wall, close
        for _ in range(10):
            w.pulse('W', 600)
        self.assertGreater(w.bumps, 0)
        self.assertLess(w.y, 4.0)

    def test_rover_cannot_drive_into_dog(self):
        w = world(dog=rs.Dog(1.6, 2.0, math.pi / 2, [('stand', 1e9, False)]))
        for _ in range(10):
            w.pulse('W', 600)
        self.assertGreater(w.dog_contacts, 0)
        self.assertGreaterEqual(w.gap_to_dog(), -1e-6)


class CameraTest(unittest.TestCase):
    def view(self, dog):
        w = world(dog=dog, render=True)
        w.advance(0.05)
        gray, mask, _ = w.camera()
        ys, xs = np.nonzero(mask)
        w.close()
        return gray, mask, xs, ys

    def test_dog_ahead_is_centred(self):
        _, mask, xs, _ = self.view(rs.Dog(2.6, 2.0, math.pi / 2, [('stand', 1e9, False)]))
        self.assertGreater(mask.sum(), 500)
        self.assertAlmostEqual((xs.min() + xs.max()) / 2 / rs.W, 0.5, delta=0.08)

    def test_dog_on_the_left_appears_left(self):
        _, _, xs, _ = self.view(rs.Dog(2.6, 2.6, math.pi / 2, [('stand', 1e9, False)]))
        self.assertLess((xs.min() + xs.max()) / 2 / rs.W, 0.4)

    def test_lying_dog_is_shorter(self):
        _, _, _, stand = self.view(rs.Dog(3.0, 2.0, math.pi / 2, [('stand', 1e9, False)]))
        _, _, _, lie = self.view(rs.Dog(3.0, 2.0, math.pi / 2, [('sleep', 1e9)]))
        lie_h = lie.max() - lie.min()
        self.assertLess(lie_h, (stand.max() - stand.min()) * 0.8)

    def test_still_vs_wagging_pixel_change(self):
        def change(script):
            w = world(dog=rs.Dog(2.3, 2.0, 0.0, script), render=True)    # side-on, tail visible
            w.advance(0.2)
            g1, mask, _ = w.camera()
            w.advance(0.2)
            g2, _, _ = w.camera()
            ys, xs = np.nonzero(mask)
            box = (slice(ys.min() // 2, ys.max() // 2 + 1), slice(xs.min() // 2, xs.max() // 2 + 1))
            w.close()
            return float(np.mean(np.abs(g2[box] - g1[box])))
        asleep = change([('sleep', 1e9)])
        self.assertLess(asleep, 12.0, 'sensor noise alone must stay under the watcher threshold')
        walking = change([('walk', (2.3, 3.5), 0.8)])
        self.assertGreater(walking, asleep)


if __name__ == '__main__':
    unittest.main()
