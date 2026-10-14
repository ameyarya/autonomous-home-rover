"""Tests for xiao_link against a fake XIAO (MJPEG on one port, WebSocket telemetry on another).

Run: cd mac && python3 -m unittest test_xiao_link
"""
import base64
import hashlib
import json
import socket
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from xiao_link import ClockMap, StaleFrame, XiaoLink

JPEG = b'\xff\xd8fake-jpeg\xff\xd9'


class FakeXiao:
    """Device clock starts at an arbitrary uptime; frames and telemetry are stamped with it."""

    def __init__(self, frame_interval=0.03, telemetry_interval=0.05, capture_delay=0.02):
        self.boot = time.monotonic() - 12.3
        self.frame_interval = frame_interval
        self.telemetry_interval = telemetry_interval
        self.capture_delay = capture_delay  # time between capture and sending, like JPEG encode
        self.frames_on = threading.Event()
        self.frames_on.set()
        self.telemetry_on = threading.Event()
        self.telemetry_on.set()
        self.cam_ok = True
        self.stopped = threading.Event()
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), self._stream_handler())
        self.ws = socket.create_server(('127.0.0.1', 0))
        self.ws.settimeout(0.2)
        for target in (self.http.serve_forever, self._ws_accept):
            threading.Thread(target=target, daemon=True).start()

    @property
    def ports(self):
        return self.ws.getsockname()[1], self.http.server_address[1]

    def device_us(self, at=None):
        return int(((time.monotonic() if at is None else at) - self.boot) * 1e6)

    def close(self):
        self.stopped.set()
        self.http.shutdown()
        self.ws.close()

    def _stream_handler(self):
        fake = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace;boundary=frame')
                self.send_header('Transfer-Encoding', 'chunked')
                self.end_headers()
                seq = 0
                try:
                    while not fake.stopped.is_set():
                        if not fake.frames_on.is_set():
                            time.sleep(0.01)
                            continue
                        captured = fake.device_us(time.monotonic() - fake.capture_delay)
                        seq += 1
                        part = (f'\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(JPEG)}\r\n'
                                f'X-Timestamp-Us: {captured}\r\nX-Frame-Seq: {seq}\r\n\r\n').encode() + JPEG
                        self.wfile.write(f'{len(part):x}\r\n'.encode() + part + b'\r\n')
                        self.wfile.flush()
                        time.sleep(fake.frame_interval)
                except OSError:
                    pass

            def log_message(self, *args):
                pass

        return Handler

    def _ws_accept(self):
        while not self.stopped.is_set():
            try:
                conn, _ = self.ws.accept()
            except (TimeoutError, OSError):
                continue
            threading.Thread(target=self._ws_serve, args=(conn,), daemon=True).start()

    def _ws_serve(self, conn):
        try:
            request = conn.recv(4096).decode()
            key = next(line.split(':', 1)[1].strip() for line in request.split('\r\n')
                       if line.lower().startswith('sec-websocket-key'))
            accept = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
            conn.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                         b'Sec-WebSocket-Accept: ' + accept + b'\r\n\r\n')
            seq = 0
            while not self.stopped.is_set():
                if self.telemetry_on.is_set():
                    seq += 1
                    payload = json.dumps({'type': 'telemetry', 'seq': seq, 't_us': self.device_us(),
                                          'cam_ok': self.cam_ok, 'rssi': -40, 'fw': 'fake'}).encode()
                    conn.sendall(bytes([0x81, len(payload)]) + payload if len(payload) < 126 else
                                 bytes([0x81, 126]) + len(payload).to_bytes(2, 'big') + payload)
                time.sleep(self.telemetry_interval)
        except OSError:
            pass
        finally:
            conn.close()


class ClockMapTest(unittest.TestCase):
    def test_uses_smallest_delay_sample(self):
        clock = ClockMap()
        clock.observe(1_000_000, 101.050)  # 50 ms delay
        clock.observe(2_000_000, 102.010)  # 10 ms delay
        clock.observe(3_000_000, 103.200)  # 200 ms delay
        self.assertAlmostEqual(clock.to_mac(2_500_000), 102.510)

    def test_device_restart_clears_mapping(self):
        clock = ClockMap()
        clock.observe(50_000_000, 150.0)
        clock.observe(1_000_000, 200.0)  # uptime went backwards: rebooted
        self.assertEqual(clock.restarts, 1)
        self.assertAlmostEqual(clock.to_mac(1_000_000), 200.0)

    def test_unmapped_until_first_sample(self):
        self.assertIsNone(ClockMap().to_mac(123))


class XiaoLinkTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeXiao()
        http_port, stream_port = self.fake.ports
        self.link = XiaoLink('127.0.0.1', max_frame_age_s=0.3, max_telemetry_age_s=0.4,
                             http_port=http_port, stream_port=stream_port, io_timeout=1.0).start()

    def tearDown(self):
        self.link.stop()
        self.fake.close()

    def test_fresh_frame_has_measured_age(self):
        frame = self.link.fresh_frame(timeout=3)
        self.assertEqual(frame.jpeg, JPEG)
        # Fake device captures 20 ms before sending; mapped age must reflect that, not zero.
        self.assertGreaterEqual(frame.age(frame.received), 0.015)
        self.assertLess(frame.age(), 0.3)
        self.assertEqual(self.link.health()['state'], 'ok')

    def test_frames_stopping_makes_link_stale(self):
        self.link.fresh_frame(timeout=3)
        self.fake.frames_on.clear()
        stopped = time.monotonic()
        with self.assertRaises(StaleFrame) as raised:
            while True:
                self.link.fresh_frame(timeout=0.05)
                self.assertLess(time.monotonic() - stopped, 1.0, 'stale frame never detected')
        detected = time.monotonic() - stopped
        self.assertIn('frame stale', str(raised.exception))
        self.assertLess(detected, 0.3 + 0.15)
        self.assertEqual(self.link.health()['state'], 'stale')
        self.fake.frames_on.set()
        self.link.fresh_frame(timeout=3)  # recovers

    def test_telemetry_loss_makes_frames_unusable(self):
        self.link.fresh_frame(timeout=3)
        self.fake.telemetry_on.clear()
        time.sleep(0.6)
        with self.assertRaises(StaleFrame) as raised:
            self.link.fresh_frame(timeout=0.1)
        self.assertIn('telemetry stale', str(raised.exception))

    def test_camera_fault_blocks_frames(self):
        self.link.fresh_frame(timeout=3)
        self.fake.cam_ok = False
        time.sleep(0.2)
        with self.assertRaises(StaleFrame) as raised:
            self.link.fresh_frame(timeout=0.1)
        self.assertIn('camera fault', str(raised.exception))

    def test_waits_for_frame_captured_after_request(self):
        self.link.fresh_frame(timeout=3)
        requested = time.monotonic()
        frame = self.link.fresh_frame(captured_after=requested, timeout=3)
        self.assertGreaterEqual(frame.captured, requested)

    def test_save_writes_jpeg_and_sidecar(self):
        import tempfile
        from pathlib import Path
        frame = self.link.fresh_frame(timeout=3)
        with tempfile.TemporaryDirectory() as directory:
            path = self.link.save(frame, directory)
            self.assertEqual(path.read_bytes(), JPEG)
            meta = json.loads(path.with_suffix('.json').read_text())
            self.assertEqual(meta['seq'], frame.seq)
            self.assertEqual(Path(path).suffix, '.jpg')


if __name__ == '__main__':
    unittest.main()
