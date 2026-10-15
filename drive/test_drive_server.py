"""Run: cd drive && python3 -m unittest test_drive_server"""
import threading
import time
import unittest

import drive_server
from drive_server import DriveState


class FakeTx:
    def __init__(self):
        self.calls = []

    def stop(self):
        self.calls.append('X')

    def pulse(self, key, ms, abort=None):
        self.calls.append(f'{key}{ms}')
        end = time.monotonic() + ms / 1000
        while time.monotonic() < end and not (abort and abort.is_set()):
            time.sleep(0.01)
        self.stop()


class DriveStateTest(unittest.TestCase):
    def setUp(self):
        self.tx = FakeTx()
        self.s = DriveState(self.tx)

    def test_pulse_refused_when_disarmed(self):
        with self.assertRaises(PermissionError):
            self.s.pulse('W', 100)
        self.assertEqual(self.tx.calls, [])

    def test_pulse_is_capped(self):
        self.s.arm(True)
        result = self.s.pulse('S', 5000)
        self.assertEqual(result['ms'], drive_server.PULSE_LIMITS_MS['S'])

    def test_lease_expires_and_stops(self):
        drive_server.LEASE_S, old = 0.3, drive_server.LEASE_S
        try:
            threading.Thread(target=self.s.watchdog, daemon=True).start()
            self.s.arm(True)
            self.assertTrue(self.s.armed)
            time.sleep(0.8)
            self.assertFalse(self.s.armed)
            self.assertEqual(self.s.last['why'], 'arm lease expired')
            with self.assertRaises(PermissionError):
                self.s.pulse('W', 100)
        finally:
            drive_server.LEASE_S = old

    def test_stop_aborts_running_pulse(self):
        self.s.arm(True)
        started = time.monotonic()
        t = threading.Thread(target=self.s.pulse, args=('W', 600))
        t.start()
        time.sleep(0.1)
        threading.Thread(target=self.s.stop).start()
        t.join(2)
        self.assertLess(time.monotonic() - started, 0.4)
        self.assertFalse(self.s.armed)

    def test_bad_key(self):
        self.s.arm(True)
        with self.assertRaises(ValueError):
            self.s.pulse('Q', 100)


if __name__ == '__main__':
    unittest.main()
