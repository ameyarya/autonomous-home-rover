"""Run: cd mac && python3 -m unittest test_shadow"""
import json
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import shadow


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


def sec(l, c, r):
    return {k: {'cm': v} for k, v in (('left', l), ('centre', c), ('right', r))}


class ShadowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        shadow.PARAMS_FILE = self.tmp / 'explore_params.json'          # never the real settings
        self.state = {'/status': {'holding': None, 'imu': {'heading_deg': 3.0, 'rate_dps': 0.5, 'bumps': 0},
                                  'odometer': {'x_cm': 10}}}
        self.server = serve(self.state)
        url = f'http://127.0.0.1:{self.server.server_port}'
        self.depth = {'/status': {'ok': True, 'sectors': sec(150, 80, 60), 'stop_for': []}}
        self.dserver = serve(self.depth)
        self.link = Link()
        self.s = shadow.Shadow(self.link, url, f'http://127.0.0.1:{self.dserver.server_port}', self.tmp / 'shadow',
                               free_bytes=lambda: 50e9)

    def tearDown(self):
        self.server.shutdown()
        self.dserver.shutdown()

    def test_records_only_while_you_drive(self):
        self.s.set(True)
        self.assertIsNone(self.s.step())                                 # not holding a key
        self.state['/status']['holding'] = 'W'
        sample = self.s.step()
        self.assertEqual((sample['key'], sample['action']), ('W', 'forward'))
        self.assertEqual(sample['sectors'], {'left': 150, 'centre': 80, 'right': 60})
        self.assertEqual(sample['explorer'], 'left')                     # 80 < 100: it would arc toward the open left
        self.assertTrue((self.tmp / 'shadow' / self.s.session / 'f00000.jpg').exists())
        self.assertIsNone(self.s.step())                                 # same frame again

    def test_report_tunes_forward_threshold_to_your_driving(self):
        self.s.set(True)
        rows = [{'i': i, 'key': 'W', 'action': 'forward', 'sectors': {'left': 120, 'centre': 80, 'right': 120},
                 'stop_for': []} for i in range(40)]
        rows += [{'i': 40 + i, 'key': 'S', 'action': 'back', 'sectors': {'left': 40, 'centre': 45, 'right': 40},
                  'stop_for': []} for i in range(20)]
        (self.tmp / 'shadow' / self.s.session / 'samples.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
        rep = self.s.report()
        self.assertEqual(rep['samples'], 60)
        self.assertLess(rep['agree_pct'], 50)                            # explorer wants to turn at 80 cm
        self.assertLessEqual(rep['suggested']['FORWARD_CM'], 80)
        self.assertEqual(rep['suggested_agree_pct'], 100.0)
        self.assertEqual(rep['confusion']['forward']['left'], 40)

    def test_apply_keeps_safety_floors(self):
        with self.assertRaises(ValueError):
            shadow.apply_params({'FORWARD_CM': 40, 'TURN_CM': 35, 'SIDE_CM': 80})
        p = shadow.apply_params({'FORWARD_CM': 80, 'TURN_CM': 50, 'SIDE_CM': 100})
        self.assertEqual(p['FORWARD_CM'], 80)
        self.assertTrue(shadow.PARAMS_FILE.exists())


if __name__ == '__main__':
    unittest.main()
