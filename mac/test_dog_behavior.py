"""Run: cd mac && python3 -m unittest test_dog_behavior"""
import unittest

from dog_behavior import Behavior, Dog, Settings


def still_dog(x=50, height=40, pixels=2.0):
    return Dog(x=x, y=50, height=height, confidence=0.9, pixel_change=pixels)


class Sim:
    """Feeds frames at 5 Hz."""

    def __init__(self, **settings):
        self.b = Behavior(Settings(**settings))
        self.t = 0.0
        self.actions = []

    def run(self, seconds, dog_at, model_state=None, **kw):
        decision = None
        for _ in range(int(seconds * 5)):
            self.t += 0.2
            dog = dog_at(self.t) if callable(dog_at) else dog_at
            decision = self.b.update(self.t, dog, model_state, **kw)
            self.actions.append(decision.action)
        return decision


def walking(t):
    return Dog(x=30 + (t * 10) % 40, y=50, height=40, confidence=0.9, pixel_change=20)


class BehaviorTest(unittest.TestCase):
    def test_parked_dog_still_stays_parked(self):
        sim = Sim()
        d = sim.run(10, still_dog())
        self.assertEqual((d.state, d.action), ('parked', 'hold'))

    def test_dog_movement_starts_following(self):
        sim = Sim()
        sim.run(2, still_dog())
        d = sim.run(1, walking)
        self.assertEqual(d.state, 'following')

    def test_tail_wag_counts_as_movement(self):
        sim = Sim()
        d = sim.run(2, still_dog(pixels=30))  # box not moving, pixels inside changing
        self.assertEqual(d.state, 'following')

    def test_follow_actions_center_then_standoff(self):
        sim = Sim()
        sim.run(1, walking)
        self.assertEqual(sim.run(0.2, still_dog(x=20)).action, 'turn_left')
        self.assertEqual(sim.run(0.2, still_dog(x=80)).action, 'turn_right')
        self.assertEqual(sim.run(0.2, still_dog(height=15)).action, 'forward')
        self.assertEqual(sim.run(0.2, still_dog(height=70)).action, 'back')

    def test_close_and_off_centre_backs_up_before_turning(self):
        sim = Sim()
        sim.run(1, walking)
        self.assertEqual(sim.run(0.2, still_dog(x=15, height=70)).action, 'back',
                         'a forward arc would drive into him')
        self.assertEqual(sim.run(0.2, still_dog(x=15, height=40)).action, 'turn_left')

    def test_scans_toward_side_dog_left(self):
        for x, expected in ((20, 'scan_left'), (80, 'scan_right')):
            sim = Sim()
            sim.run(1, walking)
            sim.run(0.4, still_dog(x=x))
            d = sim.run(5, None)
            self.assertEqual((d.state, d.action), ('searching', expected))

    def test_parks_when_dog_sleeps_for_a_while(self):
        sim = Sim(sleep_after_s=60)
        sim.run(1, walking)
        d = sim.run(30, still_dog(), model_state='sleeping')
        self.assertEqual(d.state, 'following', 'must not park before sleep_after_s')
        d = sim.run(40, still_dog(), model_state='sleeping')
        self.assertEqual(d.state, 'parked')
        self.assertEqual(sim.actions.count('park'), 1, 'park is issued once, then the rover holds')

    def test_still_but_awake_does_not_park(self):
        sim = Sim(sleep_after_s=60)
        sim.run(1, walking)
        d = sim.run(200, still_dog(), model_state='active')
        self.assertEqual(d.state, 'following')

    def test_resting_parks_after_longer(self):
        sim = Sim(sleep_after_s=60, resting_factor=2)
        sim.run(1, walking)
        self.assertEqual(sim.run(90, still_dog(), model_state='resting').state, 'following')
        self.assertEqual(sim.run(40, still_dog(), model_state='resting').state, 'parked')

    def test_wakes_up_after_parking(self):
        sim = Sim(sleep_after_s=10)
        sim.run(1, walking)
        sim.run(15, still_dog(), model_state='sleeping')
        self.assertEqual(sim.b.state, 'parked')
        self.assertEqual(sim.run(1, walking).state, 'following')

    def test_lost_dog_searches_then_parks(self):
        sim = Sim(search_limit_s=20)
        sim.run(1, walking)
        self.assertEqual(sim.run(2, None).action, 'hold')
        d = sim.run(2, None)
        self.assertEqual(d.state, 'searching')
        self.assertIn(d.action, ('scan_left', 'scan_right'))
        d = sim.run(21, None)
        self.assertEqual(d.state, 'parked')
        self.assertIn('park', sim.actions)

    def test_search_finds_dog(self):
        sim = Sim()
        sim.run(1, walking)
        sim.run(5, None)
        self.assertEqual(sim.run(0.2, still_dog()).state, 'following')

    def test_dog_charging_backs_off(self):
        sim = Sim()
        sim.run(1, walking)
        d = sim.run(1, lambda t: Dog(x=50, y=50, height=30 + (t % 10) * 40, confidence=.9, pixel_change=20))
        self.assertEqual(d.action, 'back')
        self.assertIn('closing fast', d.reason)

    def test_borderline_pixels_do_not_flicker(self):
        sim = Sim()
        flips, last = 0, None
        for i in range(50):  # 10 s of readings hovering around the 12-level threshold
            sim.t += 0.2
            sim.b.update(sim.t, still_dog(pixels=13 if i % 2 else 11))
            moving = sim.b.motion.moving
            flips += last is not None and moving != last
            last = moving
        self.assertLessEqual(flips, 1)

    def test_still_needs_confirmation(self):
        sim = Sim(still_confirm_s=2.0)
        sim.run(1, walking)
        sim.run(1, still_dog())
        self.assertTrue(sim.b.motion.moving, 'one quiet second is not yet still')
        sim.run(3, still_dog())
        self.assertFalse(sim.b.motion.moving)

    def test_stale_camera_stops(self):
        sim = Sim()
        sim.run(1, walking)
        d = sim.b.update(sim.t + 0.2, None, camera_ok=False)
        self.assertEqual(d.action, 'stop')

    def test_motion_ignored_while_rover_moving(self):
        sim = Sim()
        d = sim.run(2, walking, rover_moving=True)
        self.assertEqual(d.state, 'parked')


if __name__ == '__main__':
    unittest.main()
