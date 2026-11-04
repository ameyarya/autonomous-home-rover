"""Run: cd mac && python3 -m unittest test_explore   (fake services; no rover)"""
import time
import unittest

import explore


def sectors(left, centre, right):
    return {k: {'cm': v} for k, v in (('left', left), ('centre', centre), ('right', right))}


class ChooseTest(unittest.TestCase):
    def test_choices(self):
        self.assertEqual(explore.choose({'left': 300, 'centre': 300, 'right': 300}), ('forward', 25))
        self.assertEqual(explore.choose({'left': 300, 'centre': 80, 'right': 40}), ('left', 15))
        self.assertEqual(explore.choose({'left': 40, 'centre': 80, 'right': 200}), ('right', 15))
        self.assertEqual(explore.choose({'left': 40, 'centre': 40, 'right': 40}), ('back', 25))


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.depth = {'ok': True, 'sectors': sectors(300, 300, 300), 'stop_for': []}
        self.calls = []
        self.real = explore.call

        def fake(url, body=None, timeout=20):
            self.calls.append(url.rsplit('/', 1)[-1])
            if url.endswith('/status') and explore.DEPTH in url:
                return self.depth
            if url.endswith('/move'):
                end = time.monotonic() + 1.0                # a 1 s move; ends early once stopped
                while time.monotonic() < end and 'stop' not in self.calls:
                    time.sleep(0.02)
                return {'ended': 'aborted' if 'stop' in self.calls else 'done'}
            return {}
        explore.call = fake

    def tearDown(self):
        explore.call = self.real

    def test_dog_stops_a_move(self):
        g = explore.Guard()
        import threading
        threading.Timer(0.3, lambda: self.depth.update(stop_for=['dog'])).start()
        t = time.monotonic()
        g.move('forward', 25)
        self.assertLess(time.monotonic() - t, 0.8)
        self.assertEqual(g.why, 'saw dog')
        self.assertIn('stop', self.calls)

    def test_close_obstacle_stops_forward_not_back(self):
        self.depth['sectors'] = sectors(300, 20, 300)
        g = explore.Guard()
        g.move('back', 25)
        self.assertIsNone(g.why)                            # backing away from it is fine
        g.move('forward', 25)
        self.assertEqual(g.why, 'centre 20 cm')

    def test_clear_move_runs_to_the_end(self):
        g = explore.Guard()
        self.assertEqual(g.move('forward', 25)['ended'], 'done')
        self.assertNotIn('stop', self.calls)

    def test_waits_for_the_dog_to_leave(self):
        self.depth['stop_for'] = ['dog']
        import threading
        threading.Timer(0.6, lambda: self.depth.update(stop_for=[])).start()
        self.assertIsNone(explore.wait_for_clear_view())


if __name__ == '__main__':
    unittest.main()
