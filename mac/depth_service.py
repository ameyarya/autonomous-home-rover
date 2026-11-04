"""Camera-only obstacle sensing: Depth Anything V2 (Small) on the dashboard's live frames.

Run (the dashboard's Services start it the same way): the Mini-T detector Python, which has torch,
    python depth_service.py            (HTTP on 127.0.0.1:8774; reads http://127.0.0.1:8010/live.jpg)

  GET /status      fps, model ms, frame age, and the latest per-sector reading
  GET /depth.jpg   camera frame | depth colour map with the floor (green) and obstacles (red)
  GET /map.png     top-down map built from the free floor and the rover's pose (floor_map.py)
  GET /map         map stats;   POST /map/reset   start a new map
Living things: YOLO11s (COCO, mac/weights/yolo11s.pt) runs every other frame; a dog, cat or person
seen in the last SEEN_HOLD_S is reported in "stop_for", and the explorer never drives while it isn't
empty. Boxes are drawn on the depth view.
The map takes a view only while the rover stands still (pose and picture then agree), at most
MAP_EVERY_S, and only with the heading stick online.

The model gives relative inverse depth with an unknown scale and offset, so this doesn't trust
its numbers directly. It assumes the bottom of the picture is floor and fits how floor depth changes
with height in the image (a flat floor is a straight line in inverse depth). Anything nearer than
the floor would be at its height is something standing on the floor; anything much farther than the floor would be (below
the horizon) isn't floor either: a window, glass, a mirror or bright fabric (the balcony blinds
read as "far" and the rover drove into them, 2026-11-04), so it blocks the same way. Each column's free floor runs from the bottom up to the
first such thing, and the camera's fixed geometry turns that row into a distance:
D = K / (row - HORIZON), both scaled to the image height. Measured 2026-11-04 from the run into the
blinds (the floor's end in 18 frames against the odometer, rms 3.9 px): horizon at 0.4265 of the
height, K = 3991 px*cm at 480 rows, so the bottom edge of the picture is ~15 cm ahead. (The first
version took the horizon from the farthest thing in view; facing the blinds that put it at the
blinds' foot, so the floor read "clear to the horizon" up to contact.) Re-measure after moving the
camera. Left / centre / right report their nearest free distance.

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

import floor_map

ROOT = Path(__file__).resolve().parent
WEIGHTS = ROOT / 'weights'
FLOOR_ROWS = 0.15        # bottom share of the picture assumed to be floor, for the fit
TOLERANCE = 0.06         # nearer than the floor line by this share of the visible floor's range = obstacle
FAR_TOLERANCE = 0.12     # farther than the floor line by this share (below the horizon) = not floor
FAR_MARGIN = 0.05        # ...ignoring this share of the floor rows just under the horizon (noisy)
HORIZON_FRAC = 0.4265    # camera geometry (see above)
K_PX_CM = 3991.0         # at 480 rows
MIN_RUN = 4              # obstacle rows needed in a column (ignores specks)
SECTORS = ('left', 'centre', 'right')
MAX_CM = 300.0           # "clear to the horizon" is reported as this
BLOCKED_CM, NEAR_CM = 35.0, 80.0
MAP_EVERY_S = 0.5
MAP_SETTLE_S = 0.6       # after a move, wait this long before mapping (picture and pose settle)
MAP_COLUMNS = 64
WATCH = {16: 'dog', 15: 'cat', 0: 'person'}   # COCO class ids
WATCH_CONF = {'dog': 0.35, 'cat': 0.35, 'person': 0.5}
SEEN_HOLD_S = 1.5
SEEN_MIN = 2             # detections within SEEN_HOLD_S before it counts (one-frame false "person" seen)
SMOOTH_N = 5             # sectors report the median of this many frames (single-frame glitches seen)


def fit_floor(disp):
    """Floor line disp = a * row + b from the median of each bottom row (centre half of the width)."""
    h, w = disp.shape
    rows = np.arange(int(h * (1 - FLOOR_ROWS)), h)
    med = np.median(disp[rows, w // 4: 3 * w // 4], axis=1)
    a, b = np.polyfit(rows, med, 1)
    return float(a), float(b)


def floor_cm(row, h, horizon):
    return K_PX_CM * h / 480.0 / max(1e-6, row - horizon)


def analyse(disp):
    """disp: relative inverse depth (bigger = nearer), H x W. Returns the reading, an obstacle mask
    and each column's free floor distance in cm."""
    h, w = disp.shape
    a, b = fit_floor(disp)
    horizon = int(HORIZON_FRAC * h)
    if a <= 0 or a * (h - 1) + b <= a * horizon + b:     # the bottom isn't floor getting nearer downwards
        return {'ok': False, 'why': 'no floor at the bottom of the picture'}, np.zeros_like(disp, bool), None
    rows = np.arange(h)[:, None]
    span = a * (h - 1 - horizon)                                # disparity range of the visible floor
    floor = a * rows + b
    near = disp > floor + TOLERANCE * span
    far = (disp < floor - FAR_TOLERANCE * span) & (rows > horizon + FAR_MARGIN * (h - 1 - horizon))
    mask = (near | far) & (rows > horizon)
    free_cm = np.full(w, MAX_CM)
    for x in range(w):               # scan up from the bottom to the first run of obstacle rows
        run = 0
        for r in range(h - 1, horizon, -1):
            run = run + 1 if mask[r, x] else 0
            if run >= MIN_RUN:
                # (Tried: placing the distance at the floor row matching the obstacle's depth. It fixed
                # rendered sim images, 47 cm read 100 -> 70, but made real ones read about half the true
                # distance, so it's out: tune on real frames, not renders.)
                free_cm[x] = min(MAX_CM, floor_cm(r + MIN_RUN - 1, h, horizon))
                break
    sectors = {}
    for name, part in zip(SECTORS, np.array_split(free_cm, 3)):
        cm = float(np.percentile(part, 10))                     # the near end of the sector
        sectors[name] = {'cm': round(cm), 'state': state_for(cm)}
    return {'ok': True, 'horizon_row': horizon, 'sectors': sectors}, mask, free_cm


def state_for(cm):
    return 'blocked' if cm < BLOCKED_CM else 'near' if cm < NEAR_CM else 'clear'


def smooth(history):
    """Median per sector over recent readings (each {'left': cm, ...})."""
    return {name: {'cm': round(float(np.median([h[name] for h in history]))),
                   'state': state_for(float(np.median([h[name] for h in history])))} for name in SECTORS}


def overlay(frame, disp, mask, free_cm, horizon):
    import cv2
    v = disp - disp.min()
    v = (255 * v / max(float(v.max()), 1e-6)).astype(np.uint8)
    colour = cv2.applyColorMap(v, cv2.COLORMAP_INFERNO)
    if free_cm is not None:
        h = disp.shape[0]
        for x in range(0, disp.shape[1], 4):                    # free floor in green, obstacles red
            top = int(horizon + K_PX_CM * h / 480.0 / free_cm[x])
            cv2.line(colour, (x, h - 1), (x, top), (60, 200, 60), 1)
        colour[mask] = (40, 40, 230)
        cv2.line(colour, (0, horizon), (disp.shape[1], horizon), (255, 255, 255), 1)
    return np.hstack([frame, colour])


class DepthService:
    def __init__(self, source, size, device, drive='http://127.0.0.1:8769'):
        self.source, self.size, self.device, self.drive = source, size, device, drive
        self.map = floor_map.FloorMap()
        self.map_lock = threading.Lock()
        self.map_png = None
        self.mapped_at = 0.0
        self.moved_at = 0.0
        self.pose = None
        self.yolo = None
        self.seen = {}           # name -> (monotonic time, confidence, box) of the latest detection
        self.hits = {}           # name -> recent detection times
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
        from ultralytics import YOLO
        self.yolo = YOLO(str(WEIGHTS / 'yolo11s.pt'))
        self.yolo.predict(np.zeros((480, 640, 3), np.uint8), device=self.device, imgsz=416, verbose=False)

    def watch(self, img):
        """Dogs, cats and people in view (they stop the explorer)."""
        result = self.yolo.predict(img, device=self.device, imgsz=416, conf=min(WATCH_CONF.values()),
                                   verbose=False, classes=list(WATCH))[0]
        now = time.monotonic()
        for box in result.boxes:
            name = WATCH[int(box.cls)]
            if float(box.conf) < WATCH_CONF[name]:
                continue
            self.seen[name] = (now, float(box.conf), [int(v) for v in box.xyxy[0].tolist()])
            self.hits.setdefault(name, []).append(now)

    def stop_for(self):
        now = time.monotonic()
        for name in list(self.hits):
            self.hits[name] = [t for t in self.hits[name] if now - t < SEEN_HOLD_S]
        return sorted(n for n, ts in self.hits.items() if len(ts) >= SEEN_MIN)

    def frame(self):
        with urllib.request.urlopen(self.source, timeout=2) as r:
            age = r.headers.get('X-Frame-Age-Ms')
            return r.read(), r.headers.get('X-Frame-Seq'), None if age in (None, 'unknown') else int(age)

    def loop(self):
        import cv2
        import torch
        last_seq, times, history, n = None, [], [], 0
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
                n += 1
                if n % 2 == 0:
                    self.watch(img)
                reading, mask, free = analyse(disp)
                if reading['ok']:
                    history = (history + [{k: v['cm'] for k, v in reading['sectors'].items()}])[-SMOOTH_N:]
                    reading['raw'] = reading['sectors']
                    reading['sectors'] = smooth(history)
                else:
                    history = []
                self.ms = round((time.monotonic() - t) * 1000)
                if reading['ok']:
                    self.map_view(free)
                shown = img.copy()
                for name in self.stop_for():
                    _, conf, (x0, y0, x1, y1) = self.seen[name]
                    cv2.rectangle(shown, (x0, y0), (x1, y1), (0, 165, 255), 2)
                    cv2.putText(shown, f'{name} {conf:.2f}', (x0 + 4, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 165, 255), 2)
                out = overlay(shown, disp, mask, free, reading.get('horizon_row', 0))
                self.image = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.reading, self.frame_age_ms, self.error = reading, age, None
                times = [x for x in times + [time.monotonic()] if time.monotonic() - x < 2]
                self.fps = round((len(times) - 1) / max(times[-1] - times[0], 1e-6), 1) if len(times) > 1 else None
            except Exception as error:
                self.error = f'{type(error).__name__}: {error}'
                time.sleep(0.5)

    def map_view(self, free_cm):
        """Add this frame to the map if the rover is standing still with a good heading."""
        now = time.monotonic()
        try:
            with urllib.request.urlopen(self.drive + '/status', timeout=0.3) as r:
                drive = json.load(r)
        except (OSError, ValueError):
            return
        if drive.get('busy') or drive.get('holding'):
            self.moved_at = now
            return
        odo, imu = drive.get('odometer') or {}, drive.get('imu') or {}
        self.pose = (odo.get('x_cm', 0.0), odo.get('y_cm', 0.0), odo.get('heading_deg', 0.0))
        if not imu.get('online') or now - self.moved_at < MAP_SETTLE_S or now - self.mapped_at < MAP_EVERY_S:
            return
        cols = [float(c.min()) for c in np.array_split(free_cm, MAP_COLUMNS)]
        with self.map_lock:
            self.map.add_view(*self.pose, cols, MAX_CM)
            self.mapped_at = now
            self.render_map()

    def render_map(self):
        import cv2
        self.map_png = cv2.imencode('.png', self.map.render(self.pose))[1].tobytes()

    def reset_map(self):
        with self.map_lock:
            self.map.reset()
            self.render_map()
        return self.map.stats()

    def status(self):
        return {'ready': self.model is not None, 'device': self.device, 'size': self.size, 'fps': self.fps,
                'model_ms': self.ms, 'frame_age_ms': self.frame_age_ms, 'error': self.error,
                'map': self.map.stats(), 'stop_for': self.stop_for(),
                'seen': {n: {'ago_s': round(time.monotonic() - t, 1), 'conf': round(c, 2)}
                         for n, (t, c, _) in self.seen.items()}, **self.reading}


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
            if self.path == '/map.png':
                if service.map_png is None:
                    return self.reply(503, {'error': 'no map yet'})
                return self.reply(200, service.map_png, 'image/png')
            if self.path == '/map':
                return self.reply(200, service.map.stats())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if self.path == '/map/reset':
                return self.reply(200, service.reset_map())
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
    parser.add_argument('--drive', default='http://127.0.0.1:8769', help='drive server (pose for the map)')
    args = parser.parse_args()
    service = DepthService(args.source, args.size, args.device, args.drive)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(service))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'depth service on http://127.0.0.1:{args.port} (loading model)', flush=True)
    service.load()
    print('model loaded', flush=True)
    service.loop()


if __name__ == '__main__':
    main()
