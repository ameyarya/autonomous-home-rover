"""Camera-only obstacle sensing: Depth Anything V2 (Small) on the dashboard's live frames.

Run (the dashboard's Services start it the same way): the Mini-T detector Python, which has torch,
    python depth_service.py            (HTTP on 127.0.0.1:8774; reads http://127.0.0.1:8010/live.jpg)

  GET /status      fps, model ms, frame age, and the latest per-sector reading
  GET /depth.jpg   camera frame | depth colour map with the floor (green) and obstacles (red)

The model gives relative inverse depth with an unknown scale and offset, so this doesn't trust
its numbers directly. It assumes the bottom of the picture is floor and fits how floor depth changes
with height in the image (a flat floor is a straight line in inverse depth); the horizon is where
that line reaches the farthest things in view. Anything nearer than the floor would be at its
height is something standing on the floor. Each column's free floor runs from the bottom up to the
first such thing, and floor geometry turns that row into a distance: D = BOTTOM_CM * (bottom row -
horizon) / (row - horizon), BOTTOM_CM being how far ahead the bottom edge of the picture meets the
floor (measure it once; --bottom-cm). Left / centre / right report their nearest free distance.

Weights and model code live in mac/weights/ (gitignored): depth_anything_v2_vits.pth and the
model package from github.com/DepthAnything/Depth-Anything-V2 (code only).
"""
import argparse
import json
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
WEIGHTS = ROOT / 'weights'
FLOOR_ROWS = 0.15        # bottom share of the picture assumed to be floor, for the fit
TOLERANCE = 0.06         # nearer than the floor line by this share of the visible floor's range = obstacle
MIN_RUN = 4              # obstacle rows needed in a column (ignores specks)
SECTORS = ('left', 'centre', 'right')
BOTTOM_CM = 20.0         # floor distance at the bottom edge of the picture (to measure)
MAX_CM = 300.0           # "clear to the horizon" is reported as this
BLOCKED_CM, NEAR_CM = 35.0, 80.0
SMOOTH_N = 5             # sectors report the median of this many frames (single-frame glitches seen)


def fit_floor(disp):
    """Floor line disp = a * row + b from the median of each bottom row (centre half of the width)."""
    h, w = disp.shape
    rows = np.arange(int(h * (1 - FLOOR_ROWS)), h)
    med = np.median(disp[rows, w // 4: 3 * w // 4], axis=1)
    a, b = np.polyfit(rows, med, 1)
    return float(a), float(b)


def analyse(disp, bottom_cm=BOTTOM_CM):
    """disp: relative inverse depth (bigger = nearer), H x W. Returns the reading, an obstacle mask
    and each column's free floor distance in cm."""
    h, w = disp.shape
    a, b = fit_floor(disp)
    far = float(np.percentile(disp, 5))
    if a <= 0 or a * (h - 1) + b <= far:     # the bottom isn't floor getting nearer downwards
        return {'ok': False, 'why': 'no floor at the bottom of the picture'}, np.zeros_like(disp, bool), None
    horizon = int(np.clip((far - b) / a, 0, h - 2))
    rows = np.arange(h)[:, None]
    span = a * (h - 1 - horizon)                                # disparity range of the visible floor
    mask = (disp > a * rows + b + TOLERANCE * span) & (rows > horizon)
    free_cm = np.full(w, MAX_CM)
    for x in range(w):               # scan up from the bottom to the first run of obstacle rows
        run = 0
        for r in range(h - 1, horizon, -1):
            run = run + 1 if mask[r, x] else 0
            if run >= MIN_RUN:
                base = r + MIN_RUN - 1
                free_cm[x] = min(MAX_CM, bottom_cm * (h - 1 - horizon) / max(1, base - horizon))
                break
    sectors = {}
    for name, part in zip(SECTORS, np.array_split(free_cm, 3)):
        cm = float(np.percentile(part, 10))                     # the near end of the sector
        sectors[name] = {'cm': round(cm), 'state': state_for(cm)}
    return {'ok': True, 'horizon_row': horizon, 'bottom_cm': bottom_cm, 'sectors': sectors}, mask, free_cm


def state_for(cm):
    return 'blocked' if cm < BLOCKED_CM else 'near' if cm < NEAR_CM else 'clear'


def smooth(history):
    """Median per sector over recent readings (each {'left': cm, ...})."""
    return {name: {'cm': round(float(np.median([h[name] for h in history]))),
                   'state': state_for(float(np.median([h[name] for h in history])))} for name in SECTORS}


def overlay(frame, disp, mask, free_cm, horizon, bottom_cm=BOTTOM_CM):
    import cv2
    v = disp - disp.min()
    v = (255 * v / max(float(v.max()), 1e-6)).astype(np.uint8)
    colour = cv2.applyColorMap(v, cv2.COLORMAP_INFERNO)
    if free_cm is not None:
        h = disp.shape[0]
        for x in range(0, disp.shape[1], 4):                    # free floor in green, obstacles red
            top = int(horizon + bottom_cm * (h - 1 - horizon) / free_cm[x])
            cv2.line(colour, (x, h - 1), (x, top), (60, 200, 60), 1)
        colour[mask] = (40, 40, 230)
        cv2.line(colour, (0, horizon), (disp.shape[1], horizon), (255, 255, 255), 1)
    return np.hstack([frame, colour])


class DepthService:
    def __init__(self, source, size, device, bottom_cm=BOTTOM_CM):
        self.source, self.size, self.device, self.bottom_cm = source, size, device, bottom_cm
        self.model = None
        self.reading = {'ok': False, 'why': 'starting'}
        self.image = None
        self.ms = self.fps = None
        self.frame_age_ms = None
        self.error = None

    def load(self):
        import torch
        sys.path.insert(0, str(WEIGHTS / 'depth-anything-v2'))
        from depth_anything_v2.dpt import DepthAnythingV2
        model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384])
        model.load_state_dict(torch.load(WEIGHTS / 'depth_anything_v2_vits.pth', map_location='cpu',
                                         weights_only=True))
        self.model = model.to(self.device).eval()

    def frame(self):
        with urllib.request.urlopen(self.source, timeout=2) as r:
            age = r.headers.get('X-Frame-Age-Ms')
            return r.read(), r.headers.get('X-Frame-Seq'), None if age in (None, 'unknown') else int(age)

    def loop(self):
        import cv2
        import torch
        last_seq, times, history = None, [], []
        while True:
            try:
                jpeg, seq, age = self.frame()
                if seq == last_seq:
                    time.sleep(0.02)
                    continue
                last_seq = seq
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                t = time.monotonic()
                with torch.no_grad():
                    disp = self.model.infer_image(img, self.size)
                reading, mask, free = analyse(disp, self.bottom_cm)
                if reading['ok']:
                    history = (history + [{k: v['cm'] for k, v in reading['sectors'].items()}])[-SMOOTH_N:]
                    reading['raw'] = reading['sectors']
                    reading['sectors'] = smooth(history)
                else:
                    history = []
                self.ms = round((time.monotonic() - t) * 1000)
                out = overlay(img, disp, mask, free, reading.get('horizon_row', 0), self.bottom_cm)
                self.image = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.reading, self.frame_age_ms, self.error = reading, age, None
                times = [x for x in times + [time.monotonic()] if time.monotonic() - x < 2]
                self.fps = round((len(times) - 1) / max(times[-1] - times[0], 1e-6), 1) if len(times) > 1 else None
            except Exception as error:
                self.error = f'{type(error).__name__}: {error}'
                time.sleep(0.5)

    def status(self):
        return {'ready': self.model is not None, 'device': self.device, 'size': self.size, 'fps': self.fps,
                'model_ms': self.ms, 'frame_age_ms': self.frame_age_ms, 'error': self.error, **self.reading}


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, kind='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/status':
                return self.reply(200, service.status())
            if self.path == '/depth.jpg':
                if service.image is None:
                    return self.reply(503, {'error': 'no depth image yet'})
                return self.reply(200, service.image, 'image/jpeg')
            self.reply(404, {'error': 'not found'})

        def log_message(self, *args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8774)
    parser.add_argument('--source', default='http://127.0.0.1:8010/live.jpg')
    parser.add_argument('--size', type=int, default=336, help='model input size (multiple of 14)')
    parser.add_argument('--device', default='mps')
    parser.add_argument('--bottom-cm', type=float, default=BOTTOM_CM,
                        help='how far ahead the bottom edge of the picture meets the floor')
    args = parser.parse_args()
    service = DepthService(args.source, args.size, args.device, args.bottom_cm)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(service))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'depth service on http://127.0.0.1:{args.port} (loading model)', flush=True)
    service.load()
    print('model loaded', flush=True)
    service.loop()


if __name__ == '__main__':
    main()
