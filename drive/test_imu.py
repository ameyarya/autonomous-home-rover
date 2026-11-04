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

    def zero(self):
        self.step()
        self.y = 0.0


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


class HeadingHoldTest(unittest.TestCase):
    def keys(self, throttle, error, n=10, rate=0.0):
        h = imu.HeadingHold(throttle)
        return [h.key(error, rate, 0.0) for _ in range(n)]

    def test_on_course_goes_straight(self):
        self.assertEqual(set(self.keys(1, 0.5)), {'w'})
        self.assertEqual(set(self.keys(-1, -1.0)), {'s'})

    def test_small_error_mixes_straight_and_gentle(self):
        keys = self.keys(1, 3.0)                       # u = 0.3: all gentle
        self.assertEqual(set(keys), {'a'})
        keys = self.keys(1, -2.5, n=12)                # u = 0.25: mostly gentle right, some straight
        self.assertEqual(set(keys), {'w', 'd'})

    def test_bigger_error_mixes_gentle_and_full_lock(self):
        keys = self.keys(1, 6.5)                       # u = 0.65: half gentle, half full lock
        self.assertEqual(set(keys), {'a', 'A'})
        self.assertEqual(set(self.keys(1, 30.0)), {'A'})

    def test_reverse_steers_the_other_way(self):
        self.assertEqual(set(self.keys(-1, 3.0)), {'c'})
        self.assertEqual(set(self.keys(-1, -30.0)), {'Z'})

    def test_lead_stops_steering_before_target(self):
        # 3 degrees off but already turning back at 16 deg/s: go straight (or the other way)
        h = imu.HeadingHold(1)
        h.rate = 16.0
        self.assertNotIn(h.key(3.0, 16.0, 0.0), ('a', 'A'))

    def test_steady_pull_builds_trim(self):
        h = imu.HeadingHold(1)
        first = h.amount(2.0, 0.0, 0.1)
        for _ in range(30):
            later = h.amount(2.0, 0.0, 0.1)
        self.assertGreater(later, first + 0.2)
        self.assertLessEqual(h.trim, imu.TRIM_MAX)

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

    def test_holds_heading_when_the_pull_beats_gentle_steering(self):
        self.g.drift = -8.0                                # gentle alone gives only +4.8 deg/s back
        self.s.move('forward', 100)
        self.assertIn('A', self.tx.calls)
        self.assertLess(abs(self.g.y), 5.0)

    def test_forward_holds_heading_against_drift(self):
        out = self.s.move('forward', 100)                  # 2 s; unheld it would drift 8 deg
        self.assertTrue(out['gyro'])
        self.assertIn('a', self.tx.calls)                  # corrected the rightward drift, gently
        self.assertLess(abs(self.g.y), 5.0)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_straight_move_stops_when_knocked_off_course(self):
        self.g.drift = -60.0                               # something shoves it round, faster than it can steer
        out = self.s.move('forward', 100)
        self.assertTrue(out['ended'].startswith('blocked'))
        self.assertLess(out['ms'], 1500)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_straight_move_stops_on_a_bump(self):
        import threading
        self.g.drift = 0.0
        self.g.bumps = 0
        self.g.status = lambda: {'online': True, 'bumps': self.g.bumps}
        threading.Timer(0.4, lambda: setattr(self.g, 'bumps', 1)).start()
        out = self.s.move('forward', 100)
        self.assertEqual(out['ended'], 'blocked: bump')
        self.assertLess(out['ms'], 1200)

    def test_straight_move_can_hold_an_absolute_heading(self):
        self.g.drift = 0.0
        self.g.y = -6.0                                    # already 6 degrees right of where it should be
        self.s.move('forward', 100, heading=0.0)
        self.assertLess(abs(self.g.y), 2.5)                # steered back to 0, not held at -6

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

    def test_reverse_turn_turns_the_heading_the_named_way(self):
        self.g.drift = 0
        self.g.turn_dps = 22
        self.s.motion['turn_radius_cm'] = 120
        out = self.s.move('reverse_left', 20)
        self.assertEqual(out['ended'], 'done')
        self.assertEqual(set(self.tx.calls[:-1]), {'C'})
        self.assertAlmostEqual(self.g.y, 20, delta=6)
        self.tx.calls.clear()
        self.g.y = 0
        self.s.move('reverse_right', 20)
        self.assertEqual(set(self.tx.calls[:-1]), {'Z'})
        self.assertAlmostEqual(self.g.y, -20, delta=6)

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

    def test_stick_lost_stops(self):
        self.g.alive = True
        t = __import__('threading').Timer(0.25, lambda: setattr(self.g, 'alive', False))
        t.start()
        out = self.s.move('forward', 40)
        self.assertEqual(out['ended'], 'heading stick lost')
        self.assertLess(out['ms'], 600)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_arming_zeroes_the_heading_once(self):
        zeroed = []
        self.g.zero = lambda: zeroed.append(1)
        self.s.stop('test')
        self.s.arm(True)
        self.s.arm(True)                                   # lease renewal: no zero
        self.assertEqual(len(zeroed), 1)

    def test_manual_straight_driving_holds_heading(self):
        self.g.drift = -8.0
        end = time.monotonic() + 1.5
        while time.monotonic() < end:                      # the page renews the hold every 150 ms
            self.s.hold('W')
            time.sleep(0.15)
        time.sleep(0.6)
        self.assertIn('a', self.tx.calls)                  # steered back against the pull
        self.assertLess(abs(self.g.y), 5.0)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_manual_steering_passes_through_then_locks_the_new_heading(self):
        self.g.drift = 0.0
        for key, t in (('A', 0.6), ('W', 0.8)):
            end = time.monotonic() + t
            while time.monotonic() < end:
                self.s.hold(key)
                time.sleep(0.1)
        time.sleep(0.6)
        turned = self.g.y
        self.assertGreater(turned, 5.0)                    # the steer happened
        straight = [k for k in self.tx.calls if k in ('w', 'a', 'd', 'A', 'D')]
        self.assertIn('w', straight)                       # then straight on the new heading
        self.assertNotIn('d', straight)                    # without steering back to the old one

    def test_odometer_uses_measured_heading(self):
        self.g.drift = 0
        self.g.turn_dps = 22
        self.s.motion['turn_radius_cm'] = 120               # planned ~1.3 s for 30 deg
        self.s.move('left', 30)
        self.assertAlmostEqual(self.s.odometer.heading_deg, self.g.y, delta=1.0)


if __name__ == '__main__':
    unittest.main()
