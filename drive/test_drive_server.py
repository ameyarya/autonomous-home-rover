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

    def wire(self, key):
        self.calls.append(key)

    def update_app(self, source):
        self.installed_source = source
        self.calls.append('UPDATE')

    def status(self):
        return 'rover-drive-test|0'

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

    def test_hold_drives_while_renewed_then_stops(self):
        self.s.arm(True)
        for _ in range(4):
            self.s.hold('A')
            time.sleep(0.15)
        self.assertTrue(self.s.holding)
        time.sleep(drive_server.HOLD_S + 0.4)
        self.assertFalse(self.s.holding)
        self.assertEqual(self.tx.calls[-1], 'X')
        self.assertGreaterEqual(self.tx.calls.count('A'), 4)

    def test_hold_refused_when_disarmed(self):
        with self.assertRaises(PermissionError):
            self.s.hold('W')
        self.assertEqual(self.tx.calls, [])

    def test_stop_ends_hold_quickly(self):
        self.s.arm(True)
        self.s.hold('W')
        time.sleep(0.1)
        started = time.monotonic()
        self.s.stop()
        while self.s.holding and time.monotonic() - started < 1:
            time.sleep(0.01)
        self.assertLess(time.monotonic() - started, 0.3)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_autonomy_waits_after_manual_driving(self):
        self.s.arm(True)
        self.s.hold('W')
        with self.assertRaises(RuntimeError):
            self.s.pulse('W', 100)

    def test_hold_bad_key(self):
        self.s.arm(True)
        with self.assertRaises(ValueError):
            self.s.hold('Q')

    def test_calibration_install_disarms_then_updates(self):
        import calibration
        original = calibration.CAL_FILE.read_text()
        try:
            self.s.arm(True)
            values = dict(calibration.load(), motor_signs=[1, -1], steer_center=95)
            result = self.s.save_calibration(values, install=True)
            self.assertFalse(self.s.armed)
            self.assertEqual(self.tx.calls[-2:], ['X', 'UPDATE'])
            self.assertIn(b'MOTOR_SIGNS = (1, -1)', self.tx.installed_source)
            self.assertIn(b'STEER_CENTER = 95', self.tx.installed_source)
            self.assertEqual(result['saved']['steer_center'], 95)
        finally:
            calibration.CAL_FILE.write_text(original)

    def test_calibration_rejects_bad_values(self):
        import calibration
        with self.assertRaises(ValueError):
            calibration.validate(dict(calibration.load(), steer_travel=90))
        with self.assertRaises(ValueError):
            calibration.validate(dict(calibration.load(), motor_signs=[1, 0]))

    def test_bad_key(self):
        self.s.arm(True)
        with self.assertRaises(ValueError):
            self.s.pulse('Q', 100)


if __name__ == '__main__':
    unittest.main()
