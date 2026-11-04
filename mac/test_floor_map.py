"""Run: cd mac && .vision-venv/bin/python -m unittest test_floor_map"""
import math
import unittest

import numpy as np

import floor_map


def near(m, x, y):
    r, c = m.cell_of(x, y)
    return float(m.odds[r - 1:r + 2, c - 1:c + 2].max())


class FloorMapTest(unittest.TestCase):
    def test_wall_ahead_is_mapped_at_its_distance(self):
        m = floor_map.FloorMap()
        m.add_view(0, 0, 0, [100.0] * 16, 300)            # everything stops 100 cm ahead
        for _ in range(3):
            m.add_view(0, 0, 0, [100.0] * 16, 300)
        self.assertGreater(near(m, 100, 0), 1)            # obstacle 1 m ahead
        r, c = m.cell_of(50, 0)
        self.assertLess(m.odds[r, c], -1)                 # free in between
        r, c = m.cell_of(-50, 0)
        self.assertEqual(m.odds[r, c], 0)                 # behind: unknown

    def test_heading_rotates_the_view(self):
        m = floor_map.FloorMap()
        for _ in range(3):
            m.add_view(0, 0, 90, [100.0] * 16, 300)       # facing left (+y)
        self.assertGreater(near(m, 0, 100), 1)

    def test_clear_floor_adds_no_obstacles(self):
        m = floor_map.FloorMap()
        for _ in range(3):
            m.add_view(0, 0, 0, [300.0] * 16, 300)
        self.assertEqual(m.stats()['obstacle_cells'], 0)
        self.assertGreater(m.stats()['free_m2'], 0.5)

    def test_left_columns_look_left(self):
        b = floor_map.FloorMap().bearings(4)
        self.assertGreater(b[0], 0)
        self.assertLess(b[-1], 0)
        self.assertAlmostEqual(b[0] - b[-1], 2 * math.atan(0.375 / (0.5 / math.tan(math.radians(31)))), places=5)

    def test_render(self):
        m = floor_map.FloorMap()
        for _ in range(3):
            m.add_view(0, 0, 0, [120.0] * 16, 300)
        img = m.render((0, 0, 0))
        self.assertEqual(img.ndim, 3)
        self.assertGreater(img.shape[0], 50)


if __name__ == '__main__':
    unittest.main()
