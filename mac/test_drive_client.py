"""Run: cd mac && python3 -m unittest test_drive_client"""
import unittest

from drive_client import OBSERVE_S, DriveExecutor


class FakeServer:
    def __init__(self, armed=True):
        self.armed = armed
        self.posts = []

    def post(self, path, body=None, timeout=0):
        self.posts.append((path, body))
        return {'ok': True}

    def get(self, path, timeout=0):
        return {'armed': self.armed}


class DriveExecutorTest(unittest.TestCase):
    def test_disarmed_only_records(self):
        s = FakeServer(armed=False)
        r = DriveExecutor(s.post, s.get).act('turn_left', 0)
        self.assertEqual(r['why'], 'disarmed')
        self.assertEqual(s.posts, [])

    def test_pulse_then_observe_dwell(self):
        s = FakeServer()
        ex = DriveExecutor(s.post, s.get)
        r = ex.act('forward', 0)
        self.assertEqual(r['sent'], 'W 300 ms')
        self.assertEqual(s.posts[-1], ('/pulse', {'key': 'W', 'ms': 300}))
        again = ex.act('forward', ex.captured_after + OBSERVE_S / 2)
        self.assertEqual(again['why'], 'observing')
        self.assertEqual(len(s.posts), 1)
        ex.act('forward', ex.captured_after + OBSERVE_S + 0.01)
        self.assertEqual(len(s.posts), 2)

    def test_hold_sends_nothing(self):
        s = FakeServer()
        self.assertIsNone(DriveExecutor(s.post, s.get).act('hold', 0)['sent'])
        self.assertEqual(s.posts, [])

    def test_park_sends_stop_once(self):
        s = FakeServer()
        ex = DriveExecutor(s.post, s.get)
        ex.act('forward', 0)
        ex.stopped = False
        ex.act('park', 10)
        ex.act('park', 11)
        self.assertEqual([p for p, _ in s.posts].count('/stop'), 1)

    def test_scan_alternates_arc_and_reverse(self):
        s = FakeServer()
        ex = DriveExecutor(s.post, s.get)
        t = 0
        for _ in range(4):
            ex.act('scan_right', t)
            t = ex.captured_after + OBSERVE_S + 0.01
        self.assertEqual([b['key'] for _, b in s.posts], ['D', 'S', 'D', 'S'])

    def test_scan_cycle_restarts_with_arc(self):
        s = FakeServer()
        ex = DriveExecutor(s.post, s.get)
        ex.act('scan_left', 0)
        ex.act('forward', ex.captured_after + OBSERVE_S + 0.01)
        ex.act('scan_left', ex.captured_after + OBSERVE_S + 0.01)
        self.assertEqual([b['key'] for _, b in s.posts], ['A', 'W', 'A'])

    def test_observing_does_not_advance_scan(self):
        s = FakeServer()
        ex = DriveExecutor(s.post, s.get)
        ex.act('scan_left', 0)
        ex.act('scan_left', ex.captured_after)          # still observing: skipped
        ex.act('scan_left', ex.captured_after + OBSERVE_S + 0.01)
        self.assertEqual([b['key'] for _, b in s.posts], ['A', 'S'])

    def test_pulses_stay_within_server_limits(self):
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'drive'))
        from drive_client import PULSES, SCANS
        from drive_server import PULSE_LIMITS_MS
        for key, ms in list(PULSES.values()) + [p for cycle in SCANS.values() for p in cycle]:
            self.assertIn(key, PULSE_LIMITS_MS)
            self.assertLessEqual(ms, PULSE_LIMITS_MS[key])

    def test_server_down_is_safe(self):
        def down(*a, **k):
            raise OSError('connection refused')
        ex = DriveExecutor(down, lambda *a, **k: {'armed': True})
        self.assertIn('drive server', ex.act('turn_right', 0)['why'])


if __name__ == '__main__':
    unittest.main()
