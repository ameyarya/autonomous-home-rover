"""Run: cd mac && ../sim/.venv/bin/python -m unittest test_route_planner   (numpy)"""
import math
import unittest

import route_planner as P

ROOM = [{'x0': -0.1, 'y0': -0.1, 'x1': 6.1, 'y1': 0.0}, {'x0': -0.1, 'y0': 4.0, 'x1': 6.1, 'y1': 4.1},
        {'x0': -0.1, 'y0': 0.0, 'x1': 0.0, 'y1': 4.0}, {'x0': 6.0, 'y0': 0.0, 'x1': 6.1, 'y1': 4.0}]


def drive(start, moves):
    """Replay merged moves with the primitives' own geometry."""
    x, y, h = start[0], start[1], math.radians(start[2])
    prims = {name: samples[-1] for name, _, samples, _ in P.PRIMS}
    for kind, amount in moves:
        n = round(amount / (P.STEP_M * 100 if kind in ('forward', 'back') else P.ARC_DEG))
        for _ in range(n):
            dx, dy, dh = prims[kind]
            x, y, h = x + dx * math.cos(h) - dy * math.sin(h), y + dx * math.sin(h) + dy * math.cos(h), h + dh
    return x, y, math.degrees(h)


class PlannerTest(unittest.TestCase):
    def test_straight_ahead(self):
        moves, poses = P.plan(ROOM, (6, 4), (1, 2, 0), (4, 2))
        self.assertEqual(moves, [('forward', 275.0)])

    def test_goal_behind_needs_a_turn_and_arrives(self):
        moves, poses = P.plan(ROOM, (6, 4), (3, 2, 0), (1, 2))
        x, y, _ = drive((3, 2, 0), moves)
        self.assertLess(math.hypot(x - 1, y - 2), P.GOAL_M + 1e-6)
        # (straight back is a fine answer here: cheaper than a 2.65 m U-turn in a 4 m room)

    def test_wall_between_is_routed_around(self):
        wall = ROOM + [{'x0': 2.9, 'y0': 0.0, 'x1': 3.1, 'y1': 2.8}]          # a gap only at the top
        moves, poses = P.plan(wall, (6, 4), (1.5, 1.0, 90), (4.5, 1.0))
        self.assertIsNotNone(moves)
        self.assertTrue(all(p[1] > 0.2 for p in poses))
        self.assertTrue(any(p[1] > 2.8 for p in poses))                       # went through the gap

    def test_no_route_through_a_closed_door(self):
        closed = ROOM + [{'x0': 2.9, 'y0': 0.0, 'x1': 3.1, 'y1': 4.0}]
        moves, why = P.plan(closed, (6, 4), (1.5, 2, 0), (4.5, 2))
        self.assertIsNone(moves)
        self.assertIn('no way through', why)

    def test_turning_radius_is_respected(self):
        # a U-turn needs ~2.65 m: in a 1.2 m wide corridor it has to reverse at least once
        corridor = [{'x0': -0.1, 'y0': -0.1, 'x1': 8.1, 'y1': 0.0}, {'x0': -0.1, 'y0': 1.2, 'x1': 8.1, 'y1': 1.3},
                    {'x0': -0.1, 'y0': 0.0, 'x1': 0.0, 'y1': 1.2}, {'x0': 8.0, 'y0': 0.0, 'x1': 8.1, 'y1': 1.2}]
        moves, _ = P.plan(corridor, (8, 1.3), (4, 0.6, 0), (2, 0.6))
        self.assertIsNotNone(moves)
        self.assertTrue(any(k in ('back', 'reverse_left', 'reverse_right') for k, _ in moves))


if __name__ == '__main__':
    unittest.main()
