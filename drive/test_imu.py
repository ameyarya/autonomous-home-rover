"""Run: cd drive && python3 -m unittest test_imu"""
import time
import unittest

import imu
from drive_server import DriveState
from test_drive_server import FakeTx


class FakeImu:
    """A simulated rover heading: A/D (and C/Z reversing) turn it, plus a steady rightward drift."""
    def __init__(self, drift_dps=-4.0, turn_dps=16.0):
        self.y, self.r, self.drift, self.turn_dps = 0.0, 0.0, drift_dps, turn_dps
        self.key, self.t, self.alive = None, time.monotonic(), True

    def step(self):
        now = time.monotonic()
        steer = {'A': 1, 'D': -1, 'C': 1, 'Z': -1, 'a': .3, 'd': -.3, 'c': .3, 'z': -.3}.get(self.key, 0)
        moving = self.key in ('W', 'A', 'D', 'S', 'C', 'Z', 'w', 's', 'a', 'd', 'c', 'z')
        self.r = (steer * self.turn_dps + self.drift) if moving else 0.0
        self.y += self.r * (now - self.t)
        self.t = now

    def fresh(self):
        return self.alive

    def yaw(self):
        self.step()
        return self.y

    def rate(self):
        return self.r

    def status(self):
        return {'online': self.alive}


class GyroTx(FakeTx):
    def __init__(self, g):
        super().__init__()
        self.g = g

    def wire(self, key):
        self.g.step()
        self.g.key = key
        super().wire(key)

    def stop(self):
        self.g.step()
        self.g.key = None
        super().stop()


class SteerKeyTest(unittest.TestCase):
    def test_forward(self):
        self.assertEqual(imu.steer_key(1, 0.5), 'w')        # on course: wheels centred now
        self.assertEqual(imu.steer_key(1, 5), 'a')          # drifted right: gentle left
        self.assertEqual(imu.steer_key(1, -5), 'd')
        self.assertEqual(imu.steer_key(1, 20), 'A')         # far off: full lock

    def test_reverse_steers_the_other_way(self):
        self.assertEqual(imu.steer_key(-1, 0), 's')
        self.assertEqual(imu.steer_key(-1, 5), 'c')
        self.assertEqual(imu.steer_key(-1, -20), 'Z')

    def test_lead_stops_steering_before_target(self):
        # 5 degrees off but already turning left at 16 deg/s: predicted error is -1.4 deg, go straight
        self.assertEqual(imu.steer_key(1, 5, rate_dps=16), 'w')

    def test_wrap(self):
        self.assertEqual(imu.wrap(190), -170)
        self.assertEqual(imu.wrap(-190), 170)


class GyroMoveTest(unittest.TestCase):
    def setUp(self):
        self.g = FakeImu()
        self.tx = GyroTx(self.g)
        self.s = DriveState(self.tx, self.g)
        self.s.motion = {'forward_cm_s': 50, 'reverse_cm_s': 50, 'turn_radius_cm': 30, 'power': 1200,
                         'arc_forward_cm': None, 'arc_sideways_cm': None}
        self.s.power = lambda: 1200
        self.s.arm(True)

    def test_forward_holds_heading_against_drift(self):
        out = self.s.move('forward', 100)                  # 2 s; unheld it would drift 8 deg
        self.assertTrue(out['gyro'])
        self.assertIn('a', self.tx.calls)                  # corrected the rightward drift, gently
        self.assertNotIn('A', self.tx.calls)
        self.assertLess(abs(self.g.y), 5.0)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_without_hold_it_just_drives(self):
        self.s.heading_hold = False
        out = self.s.move('forward', 20)
        self.assertFalse(out['gyro'])
        self.assertEqual(set(self.tx.calls[:-1]), {'W'})   # plain timed drive

    def test_turn_stops_at_the_angle(self):
        self.g.drift = 0
        self.g.turn_dps = 22
        self.s.motion['turn_radius_cm'] = 120               # planned ~1.3 s for 30 deg
        out = self.s.move('left', 30)
        self.assertEqual(out['ended'], 'done')
        self.assertAlmostEqual(self.g.y, 30, delta=8)
        self.assertEqual(set(self.tx.calls[:-1]), {'A'})

    def test_right_turn(self):
        self.g.drift = 0
        self.g.turn_dps = 22
        self.s.motion['turn_radius_cm'] = 120               # planned ~1.3 s for 30 deg
        self.s.move('right', 30)
        self.assertAlmostEqual(self.g.y, -30, delta=8)

    def test_turn_gives_up_when_not_turning(self):
        self.g.drift, self.g.turn_dps = 0, 0
        self.s.motion['turn_radius_cm'] = 5                # planned 0.03 s for 10 deg
        out = self.s.move('left', 10)
        self.assertEqual(out['ended'], 'timeout')
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_turn_stops_when_stuck(self):
        self.g.drift, self.g.turn_dps = 0, 0
        self.s.motion['turn_radius_cm'] = 120                # planned ~4 s for 90 deg
        out = self.s.move('left', 90)
        self.assertEqual(out['ended'], 'stuck: heading not changing')
        self.assertLess(out['ms'], 2000)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_straight_move_ends_straight(self):
        self.s.move('forward', 40)
        self.assertEqual(self.tx.calls[-2], 'w')

    def test_stick_lost_stops(self):
        self.g.alive = True
        t = __import__('threading').Timer(0.25, lambda: setattr(self.g, 'alive', False))
        t.start()
        out = self.s.move('forward', 40)
        self.assertEqual(out['ended'], 'heading stick lost')
        self.assertLess(out['ms'], 600)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_odometer_uses_measured_heading(self):
        self.g.drift = 0
        self.g.turn_dps = 22
        self.s.motion['turn_radius_cm'] = 120               # planned ~1.3 s for 30 deg
        self.s.move('left', 30)
        self.assertAlmostEqual(self.s.odometer.heading_deg, self.g.y, delta=1.0)


if __name__ == '__main__':
    unittest.main()
