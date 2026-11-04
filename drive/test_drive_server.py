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

    def codes(self, codes):
        self.calls.append('CODES ' + ' '.join(codes))

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

    def test_steer_only_keys_accepted(self):
        self.s.arm(True)
        self.assertEqual(self.s.pulse('L', 5000)['ms'], 2000)
        with self.assertRaises(ValueError):
            self.s.pulse('Q', 100)

    def test_transmitter_key_frames(self):
        import rover_drive
        tx = rover_drive.Transmitter.__new__(rover_drive.Transmitter)
        sent = []
        tx.repl = sent.append
        for key in 'WADSLRZC':
            tx.wire(key)
        self.assertEqual(sent, ["wire('W')", "wire('W');wire('A')", "wire('W');wire('D')", "wire('S')",
                                "wire('A')", "wire('D')", "wire('S');wire('A')", "wire('S');wire('D')"])

    def test_transmitter_reconnects_when_the_board_restarted(self):
        import rover_drive
        tx = rover_drive.Transmitter.__new__(rover_drive.Transmitter)
        tx.reconnects = 0
        replies = [b'', b"wire('X')\r\n>>> "]                   # silent (rebooted), then fine
        class Port:
            def write(self, data): pass
        tx.ser = Port()
        tx._read_until_prompt = lambda timeout: replies.pop(0)
        tx._connect = lambda: None
        tx.repl("wire('X')")
        self.assertEqual(tx.reconnects, 1)
        replies[:] = [b'', b'']
        with self.assertRaises(RuntimeError):
            tx.repl("wire('X')")

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
            self.assertEqual(self.tx.calls[-3:], ['X', 'UPDATE', 'X'])
            self.assertIn(b'MOTOR_SIGNS = (1, -1)', self.tx.installed_source)
            self.assertIn(b'STEER_CENTER = 95', self.tx.installed_source)
            self.assertEqual(result['saved']['steer_center'], 95)
        finally:
            calibration.CAL_FILE.write_text(original)

    def test_calibration_rejects_bad_values(self):
        import calibration
        with self.assertRaises(ValueError):
            calibration.validate(dict(calibration.load(), steer_travel=91))
        with self.assertRaises(ValueError):       # centre ± travel past the servo's 0..180
            calibration.validate(dict(calibration.load(), steer_center=100, steer_travel=85))
        self.assertEqual(calibration.validate(dict(calibration.load(), steer_center=90, steer_travel=90))['steer_travel'], 90)
        with self.assertRaises(ValueError):
            calibration.validate(dict(calibration.load(), motor_signs=[1, 0]))

    def test_live_values_from_rover(self):
        import calibration
        self.assertEqual(calibration.live_values('rover-drive-0.5+cabc123 c92 t60 s950|0'),
                         {'steer_center': 92, 'steer_travel': 60, 'speed': 950, 'unsaved': False})
        self.assertTrue(calibration.live_values('rover-drive-0.5 c92 t60 s950*|1')['unsaved'])
        self.assertIsNone(calibration.live_values('rover-drive-0.4+cabc123|0'))
        original = calibration.CAL_FILE.read_text()
        try:
            self.assertFalse(calibration.sync_from_rover('rover-drive-0.5 c99 t50 s700*|0'))   # unsaved
            self.assertTrue(calibration.sync_from_rover('rover-drive-0.5 c99 t50 s700|0'))
            self.assertEqual(calibration.load()['steer_center'], 99)
        finally:
            calibration.CAL_FILE.write_text(original)

    def test_live_calibration_sends_codes_and_saves(self):
        import calibration
        original = calibration.CAL_FILE.read_text()
        try:
            self.s.installed = 'rover-drive-0.8+cabc123 c90 t40 s900|0'
            self.tx.status = lambda: 'rover-drive-0.8+cabc123 c92 t38 s920*|0'
            r = self.s.live_calibration({'steer_center': 92, 'steer_travel': 38, 'speed': 920})
            self.assertEqual(self.tx.calls[-1], 'CODES AAA AAA ADD ADD DAA DAA')
            self.assertTrue(r['live']['unsaved'])
            self.tx.status = lambda: 'rover-drive-0.8+cabc123 c92 t38 s920|0'
            self.s.live_calibration(action='save')
            self.assertEqual(self.tx.calls[-1], 'CODES DDA')
            self.assertEqual(calibration.load()['steer_center'], 92)
        finally:
            calibration.CAL_FILE.write_text(original)

    def test_live_calibration_needs_the_live_app(self):
        for installed in ('rover-drive-0.4+cabc123|0', 'rover-drive-0.6+cabc123 c90 t40 s900|0'):
            self.s.installed = installed
            with self.assertRaises(RuntimeError):
                self.s.live_calibration({'steer_center': 92})
        self.assertNotIn('CODES', ' '.join(self.tx.calls))

    def test_bad_key(self):
        self.s.arm(True)
        with self.assertRaises(ValueError):
            self.s.pulse('Q', 100)


if __name__ == '__main__':
    unittest.main()
