"""Run: sim/.venv/bin/python -m unittest discover -s sim -p 'test_apartment_sim.py'"""
import math
import unittest

import apartment_sim as A


class VirtualDoorTest(unittest.TestCase):
    def test_barrier_blocks_and_is_counted_apart_from_walls(self):
        walls = A.Walls([{'x0': 1.0, 'y0': -1, 'x1': 1.05, 'y1': 1, 'kind': 'barrier'},
                         {'x0': -1.05, 'y0': -1, 'x1': -1.0, 'y1': 1, 'kind': 'wall'}])
        r = A.SimRover(walls, 0.0, 0.0, 0.0)
        r.play = 0
        for _ in range(500):                       # drive east into the barrier
            r.command('w')
            r.step(0.01)
        self.assertLess(r.x, 1.0 - A.LENGTH_M / 2 + 0.03)
        self.assertEqual((r.barrier_hits, r.collisions), (1, 0))

    def test_rover_kinematics_match_the_measurements(self):
        walls = A.Walls([])
        r = A.SimRover(walls, 0.0, 0.0, 0.0)
        r.rng.seed(0)
        for _ in range(100):
            r.command('W')
            r.step(0.01)
        self.assertAlmostEqual(r.x, 0.25, delta=0.02)          # 25 cm/s


if __name__ == '__main__':
    unittest.main()
