"""Run: cd mac && python3 -m unittest test_capture"""
import json
import tempfile
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading

import capture


class Frame:
    def __init__(self, seq):
        self.seq, self.jpeg = seq, b'\xff\xd8jpeg'

    def age(self):
        return 0.05


class Link:
    def __init__(self):
        self.seq = 1

    def latest(self):
        return Frame(self.seq)


def serve(state):
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(state.get(self.path, {})).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = {'/status': {'busy': False, 'odometer': {'x_cm': 10, 'y_cm': 2, 'heading_deg': 5},
                                  'imu': {'heading_deg': 4.5}}}
        self.server = serve(self.state)
        url = f'http://127.0.0.1:{self.server.server_port}'
        self.link = Link()
        self.free = [50e9]
        self.r = capture.Recorder(self.link, url, url + '/nodepth', Path(self.tmp.name) / 'captures',
                                  free_bytes=lambda: self.free[0])

    def tearDown(self):
        self.server.shutdown()
        self.tmp.cleanup()

    def test_off_saves_nothing(self):
        self.state['/status']['busy'] = True
        self.assertIsNone(self.r.step())

    def test_saves_only_while_moving_and_new_frames(self):
        self.r.set(True)
        self.assertIsNone(self.r.step())                    # standing still
        self.state['/status']['busy'] = True
        path = self.r.step()
        self.assertTrue(path.exists())
        meta = json.loads(path.with_suffix('.json').read_text())
        self.assertEqual(meta['odometer']['x_cm'], 10)
        self.assertEqual(meta['imu']['heading_deg'], 4.5)
        self.assertEqual(meta['frame_seq'], 1)
        self.assertIsNone(self.r.step())                    # same frame again
        self.link.seq = 2
        self.assertIsNotNone(self.r.step())
        self.assertEqual(self.r.status()['saved'], 2)

    def test_manual_driving_counts_as_moving(self):
        self.r.set(True)
        self.state['/status']['holding'] = 'W'
        self.assertIsNotNone(self.r.step())

    def test_stops_when_the_disk_is_nearly_full(self):
        self.r.set(True)
        self.state['/status']['busy'] = True
        self.free[0] = 0.5e9
        self.assertIsNone(self.r.step())
        self.assertFalse(self.r.on)
        self.assertIn('GB free', self.r.status()['why_stopped'])


if __name__ == '__main__':
    unittest.main()
