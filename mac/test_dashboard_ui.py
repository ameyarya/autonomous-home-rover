"""Browser tests for the rover dashboard page (mac/dashboard.html + dashboard.py).

Runs the real dashboard server and page in headless Chrome against fakes: a camera link, a camera
/config endpoint, the drive server (real DriveState with a fake transmitter) and a dog watcher.

Setup once:  python3 -m venv mac/.ui-venv && mac/.ui-venv/bin/pip install playwright
Run:         cd mac && .ui-venv/bin/python -m unittest test_dashboard_ui
Uses the installed Google Chrome (no browser download).
"""
import base64
import json
import socket
import sys
import threading
import time
import unittest
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'drive'))

import dashboard        # noqa: E402
import drive_server     # noqa: E402
from playwright.sync_api import expect, sync_playwright  # noqa: E402

JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAASABIAAD/4QBMRXhpZgAATU0AKgAAAAgAAYdpAAQAAAABAAAAGgAAAAAAA6ABAAMAAAABAAEAAKACAAQAAAABAAAAEKADAAQAAAABAAAADAAAAAD/7QA4UGhvdG9zaG9wIDMuMAA4QklNBAQAAAAAAAA4QklNBCUAAAAAABDUHYzZjwCyBOmACZjs+EJ+/8AAEQgADAAQAwEiAAIRAQMRAf/EAB8AAAEFAQEBAQEBAAAAAAAAAAABAgMEBQYHCAkKC//EALUQAAIBAwMCBAMFBQQEAAABfQECAwAEEQUSITFBBhNRYQcicRQygZGhCCNCscEVUtHwJDNicoIJChYXGBkaJSYnKCkqNDU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6g4SFhoeIiYqSk5SVlpeYmZqio6Slpqeoqaqys7S1tre4ubrCw8TFxsfIycrS09TV1tfY2drh4uPk5ebn6Onq8fLz9PX29/j5+v/EAB8BAAMBAQEBAQEBAQEAAAAAAAABAgMEBQYHCAkKC//EALURAAIBAgQEAwQHBQQEAAECdwABAgMRBAUhMQYSQVEHYXETIjKBCBRCkaGxwQkjM1LwFWJy0QoWJDThJfEXGBkaJicoKSo1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoKDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uLj5OXm5+jp6vLz9PX29/j5+v/bAEMAAgICAgICAwICAwUDAwMFBgUFBQUGCAYGBgYGCAoICAgICAgKCgoKCgoKCgwMDAwMDA4ODg4ODw8PDw8PDw8PD//bAEMBAgICBAQEBwQEBxALCQsQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEP/dAAQAAf/aAAwDAQACEQMRAD8A87ooor+yD+bz/9k=')
PASSWORD = 'test-password'


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def lan_address():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(('192.168.1.1', 80))
            return s.getsockname()[0]
        except OSError:
            return None


def serve(handler, host='127.0.0.1'):
    server = dashboard.QuietServer((host, 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


# ---- fakes ----

class FakeFrame:
    jpeg, seq = JPEG, 1

    def age(self):
        return 0.05


class FakeLink:
    def __init__(self, camera_host):
        self.host = camera_host
        self.frames = True
        self.restarts = 0
        self.reset_reason = 'poweron'

    def latest(self):
        return FakeFrame() if self.frames else None

    def health(self):
        if not self.frames:
            return {'state': 'stale', 'problems': ['no frames yet'], 'host': self.host, 'stream_connected': False,
                    'fps': 0.0, 'frame_age_ms': None, 'device_restarts_seen': self.restarts, 'telemetry': {}}
        return {'state': 'ok', 'problems': [], 'host': self.host, 'stream_connected': True, 'fps': 27.5,
                'frame_age_ms': 55, 'device_restarts_seen': self.restarts,
                'telemetry': {'rssi': -41, 'fps': 28.0, 't_us': 754_000_000, 'reset_reason': self.reset_reason,
                              'mic_dbfs': -52.0, 'fw': 'test-fw', 'psram_free': 4_000_000}}


class FakeServices:
    def __init__(self):
        self.state = {n: {'running': False, 'starting': False, 'managed': False, 'exited': None, 'log_tail': None}
                      for n in ('drive', 'watcher', 'depth')}
        self.calls = []

    def status(self):
        return json.loads(json.dumps(self.state))

    def start(self, name):
        self.calls.append(('start', name))
        self.state[name].update(running=True, managed=True)

    def stop(self, name):
        self.calls.append(('stop', name))
        self.state[name].update(running=False, managed=False)


class FakeTx:
    def __init__(self):
        self.calls = []

    def stop(self):
        self.calls.append('X')

    def wire(self, key):
        self.calls.append(key)

    def pulse(self, key, ms, abort=None):
        self.calls.append(f'{key}{ms}')
        self.stop()

    def update_app(self, source):
        self.installed_source = source
        self.calls.append('UPDATE')
        import re
        self.version = re.search(rb"^VERSION = '([^']+)'", source, re.M).group(1).decode()

    def codes(self, codes):                 # live calibration, as rover app 0.6 applies them
        self.calls.append('CODES ' + ' '.join(codes))
        steps = {'AAA': ('c', 1), 'AAD': ('c', -1), 'ADA': ('t', 1), 'ADD': ('t', -1), 'DAA': ('s', 10), 'DAD': ('s', -10)}
        for code in codes:
            if code in steps:
                k, d = steps[code]
                self.live[k] += d
                self.unsaved = True
            elif code == 'DDA':
                self.saved_live, self.unsaved = dict(self.live), False
            elif code == 'DDD':
                self.live, self.unsaved = dict(self.saved_live), False

    live = {'c': 90, 't': 20, 's': 900}
    saved_live = dict(live)
    unsaved = False

    def status(self):
        v = getattr(self, 'version', 'rover-drive-0.9+c000000')
        return v + ' c%(c)d t%(t)d s%(s)d' % self.live + ('*' if self.unsaved else '') + '|0'


class FakeCamera(BaseHTTPRequestHandler):
    config = {'framesize': 'vga', 'quality': 12, 'vflip': 0, 'hmirror': 1, 'brightness': 0,
              'contrast': 0, 'saturation': 0, 'ae_level': 0, 'led': 1}
    requests = []

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if url.path != '/config':
            self.send_error(404)
            return
        changes = dict(urllib.parse.parse_qsl(url.query))
        if changes:
            FakeCamera.requests.append(changes)
            for k, v in changes.items():
                FakeCamera.config[k] = v if k == 'framesize' else int(v)
        body = json.dumps(FakeCamera.config).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


class FakeWatcher(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/status'):
            body = json.dumps({
                'state': 'following', 'intent': 'turn_left', 'reason': 'dog at 30% (left); arc left',
                'dog_visible': True, 'moving': True, 'still_for_s': 0, 'activity': 'walking to the door',
                'dog_state': 'active', 'activity_age_s': 4, 'drive': {'sent': None, 'why': 'disarmed', 'would_send': 'A 300 ms'},
                'events': [{'t': '2026-10-16T14:00:01', 'event': 'seen', 'confidence': 0.6},
                           {'t': '2026-10-16T14:00:02', 'event': 'activity', 'activity': 'walking to the door', 'state': 'active'}],
            }).encode()
            ctype = 'application/json'
        elif self.path.startswith('/frame.jpg'):
            body, ctype = JPEG, 'image/jpeg'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


class DashboardUITest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import calibration, shutil, tempfile
        cls.cal_dir = tempfile.mkdtemp()
        cls.cal_file = Path(cls.cal_dir) / 'calibration.json'
        cls.cal_file.write_text(json.dumps({'speed': 900, 'motor_signs': [1, 1], 'steer_sign': 1, 'steer_channel': 2,
                                            'steer_center': 90, 'steer_travel': 20}))   # fixed start, not the live file
        calibration.CAL_FILE = cls.cal_file            # never touch the real drive/calibration.json
        import motion
        motion.MOTION_FILE = Path(cls.cal_dir) / 'motion.json'   # nor drive/motion.json
        cls.camera = serve(FakeCamera)
        cls.tx = FakeTx()
        cls.drive_state = drive_server.DriveState(cls.tx)
        cls.drive_state.installed = cls.tx.status()     # as drive_server.main() does at startup
        cls.drive = serve(drive_server.make_handler(cls.drive_state))
        dashboard.DRIVE_URL = f'http://127.0.0.1:{cls.drive.server_port}'
        dashboard.DOG_WATCHER_URL = f'http://127.0.0.1:{free_port()}'      # nothing there: watcher offline
        dashboard.DEPTH_URL = f'http://127.0.0.1:{free_port()}'            # nor a depth service
        cls.link = FakeLink(f'127.0.0.1:{cls.camera.server_port}')
        cls.services = FakeServices()
        cls.dash = dashboard.Dashboard(cls.link, cls.services)
        cls.lan = lan_address()
        cls.server = serve(dashboard.make_handler(cls.dash, PASSWORD), '0.0.0.0')
        cls.url = f'http://127.0.0.1:{cls.server.server_port}/'
        cls.pw = sync_playwright().start()
        cls.browser = cls.pw.chromium.launch(channel='chrome', headless=True)

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.pw.stop()
        for s in (cls.server, cls.drive, cls.camera):
            s.shutdown()

    def setUp(self):
        self.link.frames = True
        self.drive_state.stop('test reset')
        self.drive_state.manual_until = 0.0     # no carry-over of the post-manual-driving pause
        self.tx.calls.clear()
        self.context = self.browser.new_context(viewport={'width': 1400, 'height': 1000})
        self.page = self.context.new_page()
        self.errors = []
        self.page.on('pageerror', lambda e: self.errors.append(str(e)))

    def tearDown(self):
        self.context.close()
        self.assertEqual(self.errors, [], 'JavaScript errors on the page')

    def open(self):
        self.page.goto(self.url)

    # ---- camera ----

    def test_live_view_hides_placeholder_when_frames_arrive(self):
        self.open()
        expect(self.page.locator('#live-empty')).to_be_hidden()
        expect(self.page.locator('#live')).to_be_visible()
        expect(self.page.locator('#live-age')).to_contain_text('55 ms old')

    def test_live_view_placeholder_comes_back_and_recovers(self):
        self.link.frames = False
        self.open()
        expect(self.page.locator('#live-empty')).to_be_visible()
        expect(self.page.locator('#cam-chip')).to_have_text('offline')
        self.link.frames = True
        expect(self.page.locator('#live-empty')).to_be_hidden(timeout=5000)
        expect(self.page.locator('#cam-chip')).to_have_text('ok')

    def test_camera_health_tiles(self):
        self.open()
        tele = self.page.locator('#tele')
        for text in ('-41 dBm', '28', '12 min', 'poweron', '-52 dBFS', 'test-fw'):
            expect(tele).to_contain_text(text)

    def test_restart_alert_and_dismiss(self):
        self.open()
        expect(self.page.locator('#alert')).to_be_hidden()
        self.link.restarts += 1
        self.link.reset_reason = 'brownout'
        expect(self.page.locator('#alert')).to_be_visible(timeout=5000)
        expect(self.page.locator('#alert-text')).to_contain_text('brownout')
        self.page.click('#alert-ok')
        expect(self.page.locator('#alert')).to_be_hidden()
        self.page.reload()                                   # dismissal sticks across reloads
        self.page.wait_for_timeout(2500)
        expect(self.page.locator('#alert')).to_be_hidden()
        self.link.restarts += 1                              # a new restart shows it again
        expect(self.page.locator('#alert')).to_be_visible(timeout=5000)
        expect(self.page.locator('#alert-text')).to_contain_text('1 new restart')

    def test_camera_settings_load_and_apply(self):
        self.open()
        self.page.click('#settings summary')
        expect(self.page.locator('[data-k=framesize]')).to_have_value('vga')
        expect(self.page.locator('[data-k=hmirror]')).to_be_checked()
        self.page.select_option('[data-k=ae_level]', '1')
        self.page.click('#cfg-apply')
        expect(self.page.locator('#cfg-status')).to_contain_text('Saved')
        self.assertEqual(FakeCamera.requests[-1]['ae_level'], '1')

    # ---- services ----

    def test_service_chips_start_and_stop(self):
        self.open()
        button = self.page.locator('#drive-svc')
        expect(button).to_have_text('Start')
        expect(self.page.locator('#drive-chip')).to_have_text('off')
        button.click()
        expect(self.page.locator('#drive-chip')).to_have_text('running')
        expect(button).to_have_text('Stop')
        button.click()
        expect(self.page.locator('#drive-chip')).to_have_text('off')
        self.assertEqual(self.services.calls[-2:], [('start', 'drive'), ('stop', 'drive')])

    # ---- drive ----

    def test_drive_server_off_says_how_to_start(self):
        old = dashboard.DRIVE_URL
        dashboard.DRIVE_URL = f'http://127.0.0.1:{free_port()}'
        try:
            self.open()
            expect(self.page.locator('#drive-status')).to_contain_text('Start it above')
        finally:
            dashboard.DRIVE_URL = old

    def test_drive_needs_arm(self):
        self.open()
        expect(self.page.locator('#drive-status')).to_contain_text('Disarmed')
        self.page.keyboard.down('w')
        expect(self.page.locator('#drive-status')).to_contain_text('Arm first')
        self.page.keyboard.up('w')
        self.assertEqual([c for c in self.tx.calls if c != 'X'], [])

    def test_hold_to_drive_then_release_stops(self):
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.keyboard.down('ArrowUp')
        time.sleep(0.6)
        self.page.keyboard.up('ArrowUp')
        time.sleep(drive_server.HOLD_S + 0.4)
        self.assertGreaterEqual(self.tx.calls.count('W'), 2)
        self.assertEqual(self.tx.calls[-1], 'X', 'release must end with a stop')
        self.assertFalse(self.drive_state.holding)

    def hold_pad(self, *dirs, seconds=0.5):
        for d in dirs:
            self.page.dispatch_event(f'.pad button[data-dir={d}]', 'pointerdown', {'pointerId': hash(d) % 1000})
        time.sleep(seconds)
        for d in dirs:
            self.page.dispatch_event(f'.pad button[data-dir={d}]', 'pointerup', {'pointerId': hash(d) % 1000})
        time.sleep(drive_server.HOLD_S + 0.4)

    def test_button_hold_drives(self):
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.hold_pad('up')
        self.assertGreaterEqual(self.tx.calls.count('W'), 2)
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_left_right_only_steer(self):
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.hold_pad('right')
        self.assertGreaterEqual(self.tx.calls.count('R'), 2)        # steer only: no W sent with it
        self.assertNotIn('W', self.tx.calls)
        self.assertNotIn('D', self.tx.calls)

    def test_keys_combine_drive_and_steer(self):
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.keyboard.down('ArrowDown')
        self.page.keyboard.down('ArrowLeft')
        time.sleep(0.5)
        self.page.keyboard.up('ArrowLeft')
        time.sleep(0.5)
        self.page.keyboard.up('ArrowDown')
        time.sleep(drive_server.HOLD_S + 0.4)
        self.assertIn('Z', self.tx.calls)                           # reverse while steering left
        self.assertIn('S', self.tx.calls)                           # then straight reverse
        self.assertEqual(self.tx.calls[-1], 'X')

    def test_escape_stops_and_disarms(self):
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.keyboard.press('Escape')
        expect(self.page.locator('#arm')).to_have_text('Arm to drive')
        self.assertFalse(self.drive_state.armed)

    # ---- calibrate page (live) ----

    def test_calibrate_live_change_save_and_undo(self):
        self.drive_state.installed = self.tx.status()
        self.page.goto(self.url + 'calibrate')
        expect(self.page.locator('#steer_center-out')).to_have_text('90')
        self.page.fill('#steer_center', '92')                    # slider
        expect(self.page.locator('#live-status')).to_contain_text('not saved yet', timeout=5000)
        expect(self.page.locator('#steer_center-out')).to_have_text('92')
        self.assertTrue(any(c.startswith('CODES AAA') for c in self.tx.calls))
        self.page.click('#undo')
        expect(self.page.locator('#steer_center-out')).to_have_text('90', timeout=5000)
        self.page.fill('#speed', '950')
        expect(self.page.locator('#speed-out')).to_have_text('950', timeout=5000)
        self.page.click('#save')
        expect(self.page.locator('#live-status')).to_have_text('Saved on the rover.', timeout=5000)
        self.assertEqual(json.loads(self.cal_file.read_text())['speed'], 950)

    def test_calibrate_hold_needs_arm(self):
        self.page.goto(self.url + 'calibrate')
        self.page.dispatch_event('[data-hold=L]', 'pointerdown')
        expect(self.page.locator('#drive-status')).to_contain_text('Arm first')
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.dispatch_event('[data-hold=L]', 'pointerdown')
        self.page.wait_for_timeout(400)
        self.page.dispatch_event('[data-hold=L]', 'pointerup')
        self.page.wait_for_timeout(600)
        self.assertIn('L', self.tx.calls)

    # ---- measured moves ----

    def test_measured_move_needs_measurements_then_drives_and_counts(self):
        self.drive_state.motion = {k: None for k in ('forward_cm_s', 'reverse_cm_s', 'turn_radius_cm', 'power')}
        self.drive_state.odometer.reset()
        self.open()
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.click('[data-move=forward][data-amount="25"]')
        expect(self.page.locator('#move-status')).to_contain_text('measure the forward speed first')
        self.drive_state.save_motion({'forward_cm_s': 50.0, 'reverse_cm_s': 40.0, 'turn_radius_cm': 40.0, 'power': 900})
        self.page.click('[data-move=forward][data-amount="25"]')                  # 0.5 s
        expect(self.page.locator('#move-status')).to_contain_text('drove', timeout=5000)
        expect(self.page.locator('#odo-dist')).to_have_text('25 cm')
        self.assertEqual(self.tx.calls[-1], 'X')
        self.page.click('#odo-reset')
        expect(self.page.locator('#odo-dist')).to_have_text('0 cm')

    def test_heading_card_shows_the_stick_and_toggles_hold(self):
        class Stick:
            zeroed = False
            def status(self):
                return {'online': True, 'calibrated': True, 'heading_deg': 30.4, 'yaw_deg': 30.4, 'rate_dps': 1.0,
                        'still': True, 'bumps': 2, 'battery': 88, 'age_ms': 20}
            def fresh(self):
                return False                     # moves stay timed in this test
            def zero(self):
                Stick.zeroed = True
        self.drive_state.imu = Stick()
        try:
            self.open()
            expect(self.page.locator('#imu-head')).to_have_text('+30°')
            expect(self.page.locator('#imu-info')).to_contain_text('battery 88%')
            self.page.click('#imu-hold')
            for _ in range(50):
                if not self.drive_state.heading_hold:
                    break
                time.sleep(0.05)
            self.assertFalse(self.drive_state.heading_hold)
            self.page.click('#imu-zero')
            for _ in range(50):
                if Stick.zeroed:
                    break
                time.sleep(0.05)
            self.assertTrue(Stick.zeroed)
        finally:
            self.drive_state.imu = None
            self.drive_state.heading_hold = True

    def test_depth_card_shows_sectors_and_starts_the_service(self):
        reading = {'ready': True, 'ok': True, 'fps': 9.5, 'model_ms': 80, 'frame_age_ms': 60, 'error': None,
                   'sectors': {'left': {'cm': 300, 'state': 'clear'}, 'centre': {'cm': 28, 'state': 'blocked'},
                               'right': {'cm': 64, 'state': 'near'}}}

        class Depth(BaseHTTPRequestHandler):
            def do_GET(self):
                body = json.dumps(reading).encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):
                pass
        self.open()
        expect(self.page.locator('#depth-status')).to_contain_text('off')
        self.page.click('#depth-svc')
        self.assertIn(('start', 'depth'), self.services.calls)
        server = serve(Depth)
        old = dashboard.DEPTH_URL
        dashboard.DEPTH_URL = f'http://127.0.0.1:{server.server_port}'
        try:
            expect(self.page.locator('#sec-centre')).to_contain_text('28 cm', timeout=5000)
            expect(self.page.locator('#sec-centre')).to_have_class('blocked')
            expect(self.page.locator('#sec-left')).to_contain_text('clear')
            expect(self.page.locator('#depth-rate')).to_contain_text('9.5 fps')
        finally:
            dashboard.DEPTH_URL = old
            server.shutdown()

    def test_fits_a_laptop_window_without_scrolling(self):
        self.page.set_viewport_size({'width': 1440, 'height': 860})
        self.open()
        expect(self.page.locator('#arm')).to_be_visible()
        self.assertLessEqual(self.page.evaluate('document.scrollingElement.scrollHeight'), 860)
        for card in ('#arm', '#stop', '#imu-hold', '#depth-svc', '[data-move=back][data-amount="50"]', '#live-age'):
            box = self.page.locator(card).bounding_box()
            self.assertLessEqual(box['y'] + box['height'], 860, card)

    def test_heading_card_offline(self):
        self.open()
        expect(self.page.locator('#imu-info')).to_contain_text('offline')

    def test_calibrate_measure_motion(self):
        self.page.goto(self.url + 'calibrate')
        self.page.click('#arm')
        expect(self.page.locator('#arm')).to_contain_text('Armed')
        self.page.click('[data-test=test_reverse]')
        expect(self.page.locator('#m-status')).to_contain_text('Measure and enter', timeout=5000)
        self.page.fill('#m-fwd', '100')
        self.page.fill('#m-rev', '60')
        self.page.fill('#m-arc-f', '40')
        self.page.fill('#m-arc-s', '20')                                          # R = (1600+400)/40 = 50
        self.page.click('#m-save')
        expect(self.page.locator('#m-status')).to_contain_text('forward 50 cm/s, reverse 40 cm/s, turning radius 50 cm')
        self.page.reload()                                                       # saved values show again
        expect(self.page.locator('#m-fwd')).to_have_value('100')
        expect(self.page.locator('#m-arc-s')).to_have_value('20')

    def test_phone_width_has_no_sideways_scroll(self):
        context = self.browser.new_context(viewport={'width': 390, 'height': 844})
        page = context.new_page()
        page.goto(self.url)
        expect(page.locator('#live-empty')).to_be_hidden()
        overflow = page.evaluate('document.documentElement.scrollWidth - document.documentElement.clientWidth')
        context.close()
        self.assertLessEqual(overflow, 0)

    # ---- login (other devices) ----

    def test_lan_visitor_must_log_in_and_stop_needs_none(self):
        if not self.lan:
            self.skipTest('no LAN address')
        lan_url = f'http://{self.lan}:{self.server.server_port}/'
        self.page.goto(lan_url)
        expect(self.page.locator('input[type=password]')).to_be_visible()
        self.page.fill('input[type=password]', 'wrong')
        self.page.click('button')
        expect(self.page.locator('form p')).to_have_text('Wrong password')
        self.page.fill('input[type=password]', PASSWORD)
        self.page.click('button')
        expect(self.page.locator('#live-empty')).to_be_hidden(timeout=5000)
        stranger = self.pw.request.new_context()        # no session cookie
        try:
            self.assertEqual(stranger.post(lan_url + 'drive/stop').status, 200)
            self.assertEqual(stranger.post(lan_url + 'drive/arm', data='{"armed": true}').status, 401)
            self.assertEqual(stranger.get(lan_url + 'live.jpg').status, 401)
        finally:
            stranger.dispose()


if __name__ == '__main__':
    unittest.main()
