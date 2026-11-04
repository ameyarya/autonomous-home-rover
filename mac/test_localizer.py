"""Run: cd mac && ../sim/.venv/bin/python -m unittest test_localizer   (numpy)"""
import math
import unittest

import numpy as np

from localizer import Localizer

ROOM = [{'x0': -0.1, 'y0': -0.1, 'x1': 5.1, 'y1': 0.0}, {'x0': -0.1, 'y0': 4.0, 'x1': 5.1, 'y1': 4.1},
        {'x0': -0.1, 'y0': 0.0, 'x1': 0.0, 'y1': 4.0}, {'x0': 5.0, 'y0': 0.0, 'x1': 5.1, 'y1': 4.0},
        {'x0': 2.5, 'y0': 2.6, 'x1': 3.2, 'y1': 3.3}]                     # a table


def view(pose, columns=16):
    one = Localizer(ROOM, (5, 4), pose, n=1, spread=(0, 0, 0))
    return one.expected_cm(columns)[0]


class LocalizerTest(unittest.TestCase):
    def test_corrects_a_short_odometer(self):
        """Driving 1.6 m toward a wall, the odometer says 1.44 (10% short): the camera fixes it."""
        loc = Localizer(ROOM, (5, 4), (2.4, 1.0, 0.0), seed=1)
        true_x = 2.4
        for _ in range(8):
            true_x += 0.2
            loc.predict(0.18, 0.0, 0.0)                                    # what the odometer reports
            loc.update(view((true_x, 1.0, 0.0)))
        (x, y, h), spread = loc.estimate()
        self.assertLess(abs(x - true_x), 0.08)                              # odometry alone: 0.16 off
        self.assertLess(abs(y - 1.0), 0.1)

    def test_a_guess_inside_furniture_is_impossible(self):
        loc = Localizer(ROOM, (5, 4), (2.85, 2.95, 0.0), n=200, seed=2, spread=(0.4, 0.4, 5))
        loc.update(view((2.0, 2.0, 0.0)))
        inside = (loc.p[:, 0] > 2.55) & (loc.p[:, 0] < 3.15) & (loc.p[:, 1] > 2.65) & (loc.p[:, 1] < 3.25)
        self.assertLess(inside.mean(), 0.05)

    def test_expected_view(self):
        d = view((1.0, 2.0, 0.0), columns=3)
        self.assertAlmostEqual(d[1], 250.0, delta=1)                       # 4 m to the wall: "far"
        d = view((4.0, 2.0, 0.0), columns=3)
        self.assertAlmostEqual(d[1], 87.5, delta=3)                        # 1 m from the centre, the camera 12.5 cm nearer


if __name__ == '__main__':
    unittest.main()
