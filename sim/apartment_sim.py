"""The rover in a simulated copy of the apartment, behind the same interfaces as the real one.

Run:  sim/.venv/bin/python sim/apartment_sim.py [--seed 1] [--start x,y,heading]
  drive server   http://127.0.0.1:9769   the real drive/drive_server.py code (DriveState + handler),
                                         with a simulated transmitter and heading stick
  camera         http://127.0.0.1:9010/live.jpg   rendered rover camera (X-Frame-Seq, X-Frame-Age-Ms)
  truth          http://127.0.0.1:9010/truth      true pose, collisions, path, closest approach
                 POST :9010/place {"x", "y", "heading"}   put the rover somewhere (tests, batch runs)
  exact depth    http://127.0.0.1:9775/status     the depth service's left/centre/right reading, but
                 exact: 64 MuJoCo rays across the camera's view, 3 cm above the floor (over rugs).
                 Point the explorer's --depth here to test driving decisions without the depth model
                 (rendered images fool it differently from real ones; perception is tested on real
                 photos instead).
Then point the real depth service and explorer at it:
  <detector python> mac/depth_service.py --port 9774 --source http://127.0.0.1:9010/live.jpg \\
      --drive http://127.0.0.1:9769
  python3 mac/explore.py --drive http://127.0.0.1:9769 --depth http://127.0.0.1:9774 --no-capture

The apartment is sim/apartment/apartment.xml, built from the floor plan (build_apartment.py; your
home's files stay local). The rover uses the measured calibration (drive/motion.json): 25 cm/s
forward, 43 cm/s reverse, 132.5 cm full-lock radius; the receiver's key rules (rover app 0.9:
a lone W keeps a steer for 0.4 s, w centres, a/d steer 30%, X stops and centres, 0.5 s watchdog);
servo lag; front-wheel play as a slowly wandering steering offset; a gyro with a little noise and
bias. Camera: 7.5 cm up, tilted 3.8° up, 62° wide (fitted from the real run into the blinds).
Collisions stop the rover (it pushes against the wall, wheels spinning) and count as bumps.
"""
import argparse
import io
import json
import math
import random
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO / 'drive'))
import drive_server  # noqa: E402
import motion  # noqa: E402

_m = motion.scaled(motion.load(), motion.load().get('power'))
FWD_MS, REV_MS = _m['forward'] / 100, _m['reverse'] / 100     # measured: 25 and 43 cm/s
ARC_MS = _m['arc'] / 100       # measured full-lock arc speed (37 cm/s: faster than straight, as on the rover)
RADIUS_M = 1.325
WHEELBASE_M = 0.157
LENGTH_M, WIDTH_M = 0.25, 0.17          # footprint for collisions
GENTLE = 0.3
MIX_S, WATCHDOG_S = 0.4, 0.5
SERVO_TAU_S = 0.15
PLAY_RAD = math.radians(2.0)            # front-wheel play: steering offset wanders within this
CAM_H_M, CAM_PITCH_DEG, HFOV_DEG = 0.075, 3.8, 62.0
FPS = 15
SPEED_FACTOR = 1.0             # --speed-error: the rover really drives this much faster than calibrated
GYRO_BIAS_DPS = 0.02           # --gyro-drift: spread of the heading stick's leftover bias
RAY_COLUMNS, RAY_Z_M, MAX_CM = 64, 0.03, 300.0
DT = 0.01

SCENE = """<mujoco model="apartment_rover">
  <visual><global offwidth="640" offheight="480"/><quality shadowsize="2048"/>
    <map znear=".002"/>
    <headlight ambient=".45 .45 .45" diffuse=".5 .5 .5" specular="0 0 0"/></visual>
  <compiler texturedir="apartment"/>
  <include file="apartment/apartment_assets.xml"/>
  <worldbody>
    <light pos="4 4 2.3" dir="0 0 -1" diffuse=".5 .5 .5"/>
    <include file="apartment/apartment.xml"/>
    <body name="rover" mocap="true" pos="1 1 0">
      <geom type="box" size=".115 .07 .012" pos="0 0 .055" rgba=".92 .92 .9 1" contype="0" conaffinity="0"/>
      <camera name="cam" pos=".125 0 {cam_h}" xyaxes="0 -1 0 {sp} 0 {cp}" fovy="{fovy}"/>
    </body>
  </worldbody>
</mujoco>"""


def vfov(hfov_deg, aspect=4 / 3):
    return math.degrees(2 * math.atan(math.tan(math.radians(hfov_deg) / 2) / aspect))


class Walls:
    def __init__(self, rects):
        self.rects = [(r['x0'], r['y0'], r['x1'], r['y1']) for r in rects]
        self.kinds = [r.get('kind', 'wall') for r in rects]
        self.names = [r.get('name', '?') for r in rects]
        self.last_kind = self.last_name = None

    def corners(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * dx - s * dy, y + s * dx + c * dy)
                for dx, dy in ((LENGTH_M / 2, WIDTH_M / 2), (LENGTH_M / 2, -WIDTH_M / 2),
                               (-LENGTH_M / 2, -WIDTH_M / 2), (-LENGTH_M / 2, WIDTH_M / 2))]

    def hits(self, x, y, yaw):
        pts = self.corners(x, y, yaw)
        edge = [((pts[i][0] + pts[(i + 1) % 4][0]) / 2, (pts[i][1] + pts[(i + 1) % 4][1]) / 2) for i in range(4)]
        for (x0, y0, x1, y1), kind, name in zip(self.rects, self.kinds, self.names):
            for i, (px, py) in enumerate(pts + edge + [(x, y)]):
                if x0 <= px <= x1 and y0 <= py <= y1:
                    self.last_kind, self.last_name = kind, name
                    self.last_part = ('front left', 'front right', 'rear right', 'rear left', 'front', 'right side',
                                      'rear', 'left side', 'centre')[i]
                    return True
        return False

    def clearance(self, x, y):
        best = 9.9
        for x0, y0, x1, y1 in self.rects:
            dx = max(x0 - x, 0, x - x1)
            dy = max(y0 - y, 0, y - y1)
            best = min(best, math.hypot(dx, dy))
        return best


class SimRover:
    """Kinematic car with the receiver's key rules, servo lag and front-wheel play."""
    def __init__(self, walls, x, y, yaw, seed=0):
        self.walls, self.rng = walls, random.Random(seed)
        self.x, self.y, self.yaw = x, y, yaw
        self.throttle, self.target, self.steer = 0, 0.0, 0.0
        self.play = 0.0
        self.last_cmd = self.steer_at = -1e9
        self.lock = threading.Lock()
        self.collisions, self.pushing_s, self.path_m = 0, 0.0, 0.0
        self.barrier_hits = 0
        self.events = []
        self.min_clear = walls.clearance(x, y)
        self.touching = False
        self.yaw_rate = 0.0

    def command(self, key):
        """One key as the transmitter's wire() sends it (lower case = rover app 0.9 patterns)."""
        now = time.monotonic()
        with self.lock:
            self.last_cmd = now
            if key == 'X':
                self.throttle, self.target = 0, 0.0
                return
            table = {'W': (1, None), 'S': (-1, None), 'A': (1, 1), 'D': (1, -1), 'Z': (-1, 1), 'C': (-1, -1),
                     'L': (0, 1), 'R': (0, -1), 'w': (1, 0), 's': (-1, 0),
                     'a': (1, GENTLE), 'd': (1, -GENTLE), 'z': (-1, GENTLE), 'c': (-1, -GENTLE)}
            throttle, steer = table[key]
            self.throttle = throttle
            if steer is None:                       # lone W/S keeps a recent steer
                if now - self.steer_at > MIX_S:
                    self.target = 0.0
            else:
                self.target, self.steer_at = float(steer), now

    def step(self, dt):
        with self.lock:
            if time.monotonic() - self.last_cmd > WATCHDOG_S:      # the receiver's watchdog
                self.throttle, self.target = 0, 0.0
            throttle, target = self.throttle, self.target
        self.steer += (target - self.steer) * min(1.0, dt / SERVO_TAU_S)
        self.play = max(-PLAY_RAD, min(PLAY_RAD, self.play + self.rng.gauss(0, math.radians(1.5)) * math.sqrt(dt)))
        v = ((ARC_MS if abs(target) >= 1 else FWD_MS) * throttle if throttle > 0 else REV_MS * throttle) * SPEED_FACTOR
        full = math.atan(WHEELBASE_M / RADIUS_M)
        delta = self.steer * full + (self.play if throttle else 0.0)
        rate = v / WHEELBASE_M * math.tan(delta)
        nx = self.x + v * math.cos(self.yaw) * dt
        ny = self.y + v * math.sin(self.yaw) * dt
        nyaw = self.yaw + rate * dt
        if v and self.walls.hits(nx, ny, nyaw):
            if not self.touching:
                if self.walls.last_kind == 'barrier':
                    self.barrier_hits += 1
                else:
                    self.collisions += 1
                    self.events.append({'with': self.walls.last_name, 'part': self.walls.last_part,
                                        'moving': 'reverse' if throttle < 0 else 'forward',
                                        'steer': round(target, 2), 'x': round(self.x, 2), 'y': round(self.y, 2),
                                        'heading': round(math.degrees(self.yaw))})
            self.touching = True
            self.pushing_s += dt
            self.yaw_rate = 0.0
            return
        self.touching = False
        self.path_m += abs(v) * dt
        self.x, self.y, self.yaw, self.yaw_rate = nx, ny, nyaw, rate
        self.min_clear = min(self.min_clear, self.walls.clearance(nx, ny) - LENGTH_M / 2)


class SimTx:
    """Stands in for rover_drive.Transmitter."""
    def __init__(self, rover):
        self.rover = rover

    def wire(self, key):
        self.rover.command(key)

    def stop(self):
        self.rover.command('X')

    def status(self):
        return 'rover-drive-0.9+sim c130 t50 s1200|0'

    def pulse(self, key, ms, abort=None):
        end = time.monotonic() + ms / 1000
        while time.monotonic() < end and not (abort and abort.is_set()):
            self.rover.command(key)
            time.sleep(0.1)
        self.stop()

    def codes(self, codes):
        pass

    def update_app(self, source):
        pass


class SimImu:
    """Stands in for drive/imu.Imu: unwrapped heading in degrees, + = left, with noise and bias."""
    def __init__(self, rover, seed=0):
        self.rover, self.rng = rover, random.Random(seed + 1)
        self.offset = math.degrees(rover.yaw)
        self.bias_dps = self.rng.gauss(0, GYRO_BIAS_DPS)
        self.t0 = time.monotonic()
        self.bumps = 0

    def fresh(self):
        return True

    def yaw(self):
        drift = self.bias_dps * (time.monotonic() - self.t0)
        return math.degrees(self.rover.yaw) - self.offset + drift + self.rng.gauss(0, 0.05)

    def rate(self):
        return math.degrees(self.rover.yaw_rate) + self.rng.gauss(0, 0.3)

    def zero(self):
        self.offset = math.degrees(self.rover.yaw) + self.bias_dps * (time.monotonic() - self.t0)

    def status(self):
        return {'online': True, 'host': 'sim', 'error': None, 'calibrated': True,
                'heading_deg': round((self.yaw() + 180) % 360 - 180, 1), 'yaw_deg': round(self.yaw(), 2),
                'rate_dps': round(self.rate(), 2), 'still': self.rover.throttle == 0,
                'bumps': self.rover.collisions, 'battery': 100, 'age_ms': 5}


class Sim:
    def __init__(self, start, seed):
        walls = json.loads((HERE / 'apartment/apartment.json').read_text())['walls']
        self.walls = Walls(walls)
        x, y, h = start
        self.rover = SimRover(self.walls, x, y, math.radians(h), seed)
        self.imu = SimImu(self.rover, seed)
        # image x axis (0, -1, 0), image up axis (sin p, 0, cos p): looking along the rover's +x tilted
        # up by p (checked: floor-wall line at known distances gives the real rover's horizon and scale)
        p = math.radians(CAM_PITCH_DEG)
        xml = SCENE.format(cam_h=CAM_H_M, fovy=round(vfov(HFOV_DEG), 2), sp=f'{math.sin(p):.4f}',
                           cp=f'{math.cos(p):.4f}')
        (HERE / '.scene.xml').write_text(xml)
        self.model = mujoco.MjModel.from_xml_path(str(HERE / '.scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.frame, self.frame_seq, self.frame_t = None, 0, 0.0
        self.ray_data, self.ray_lock = mujoco.MjData(self.model), threading.Lock()
        self.noise = random.Random(seed + 7)
        self.rover_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'rover')

    def physics(self):
        last = time.monotonic()
        while True:
            now = time.monotonic()
            self.rover.step(now - last)
            last = now
            time.sleep(DT)

    def camera(self):
        renderer = mujoco.Renderer(self.model, 480, 640)     # in this thread: it owns the GL context
        while True:
            r = self.rover
            self.data.mocap_pos[0] = [r.x, r.y, 0]
            self.data.mocap_quat[0] = [math.cos(r.yaw / 2), 0, 0, math.sin(r.yaw / 2)]
            mujoco.mj_forward(self.model, self.data)
            renderer.update_scene(self.data, camera='cam')
            buf = io.BytesIO()
            Image.fromarray(renderer.render()).save(buf, 'JPEG', quality=80)
            self.frame, self.frame_seq, self.frame_t = buf.getvalue(), self.frame_seq + 1, time.monotonic()
            time.sleep(1 / FPS)

    def exact_depth(self, columns=RAY_COLUMNS, realistic=False):
        """Forward distance (cm) to the first thing in each of `columns` directions, as sectors.
        realistic: like a real sensor: virtual doors invisible, and noise (3 cm + 5%)."""
        with self.ray_lock:
            r = self.rover
            self.ray_data.mocap_pos[0] = [r.x, r.y, 0]
            self.ray_data.mocap_quat[0] = [math.cos(r.yaw / 2), 0, 0, math.sin(r.yaw / 2)]
            mujoco.mj_kinematics(self.model, self.ray_data)
            cam = np.array([r.x + 0.125 * math.cos(r.yaw), r.y + 0.125 * math.sin(r.yaw), RAY_Z_M])
            f = 0.5 / math.tan(math.radians(HFOV_DEG) / 2)
            cols, geom = [], np.zeros(1, np.int32)
            groups = np.array([1, 1, 1, 0, 1, 1], np.uint8) if realistic else None   # group 3 = virtual doors
            for u in (np.arange(columns) + 0.5) / columns - 0.5:
                bearing = -math.atan2(u, f)
                a = r.yaw + bearing
                d = mujoco.mj_ray(self.model, self.ray_data, cam, np.array([math.cos(a), math.sin(a), 0.0]),
                                  groups, 1, self.rover_body, geom)
                forward = d * math.cos(bearing) * 100 if d >= 0 else MAX_CM
                if realistic and forward < MAX_CM:
                    forward = max(5.0, forward + self.noise.gauss(0, 3 + 0.05 * forward))
                cols.append(min(MAX_CM, forward))
        sectors = {}
        for name, part in zip(('left', 'centre', 'right'), np.array_split(np.array(cols), 3)):
            cm = float(np.percentile(part, 10))
            sectors[name] = {'cm': round(cm), 'state': 'blocked' if cm < 35 else 'near' if cm < 80 else 'clear'}
        return {'ready': True, 'ok': True, 'frame_age_ms': 0, 'error': None, 'stop_for': [], 'sectors': sectors,
                'raw': sectors, 'map': None, 'source': 'exact (sim rays)', 'columns': [round(c, 1) for c in cols]}

    def truth(self):
        r = self.rover
        return {'x_m': round(r.x, 3), 'y_m': round(r.y, 3), 'heading_deg': round(math.degrees(r.yaw), 1),
                'collisions': r.collisions, 'barrier_hits': r.barrier_hits, 'pushing_s': round(r.pushing_s, 2),
                'path_m': round(r.path_m, 2), 'collision_events': r.events[-10:],
                'min_clearance_m': round(r.min_clear, 3), 'touching': r.touching}


def camera_handler(sim):
    class H(BaseHTTPRequestHandler):
        def reply(self, code, body, kind='application/json', headers=None):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.split('?')[0] == '/live.jpg':
                if sim.frame is None:
                    return self.reply(503, {'error': 'no frame yet'})
                age = round((time.monotonic() - sim.frame_t) * 1000)
                return self.reply(200, sim.frame, 'image/jpeg', {'X-Frame-Seq': str(sim.frame_seq),
                                                                 'X-Frame-Age-Ms': str(age)})
            if self.path == '/truth':
                return self.reply(200, sim.truth())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if self.path == '/place':
                b = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                r = sim.rover
                with r.lock:
                    r.x, r.y, r.yaw = float(b['x']), float(b['y']), math.radians(float(b['heading']))
                    r.collisions, r.pushing_s, r.path_m = 0, 0.0, 0.0
                    r.min_clear = sim.walls.clearance(r.x, r.y)
                return self.reply(200, sim.truth())
            self.reply(404, {'error': 'not found'})

        def log_message(self, *a):
            pass
    return H


def exact_depth_handler(sim):
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            path, _, query = self.path.partition('?')
            q = dict(kv.split('=', 1) for kv in query.split('&') if '=' in kv)
            body = json.dumps(sim.exact_depth(int(q.get('columns', RAY_COLUMNS)), q.get('realistic') == '1')
                              if path == '/status' else {}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            self.do_GET()

        def log_message(self, *a):
            pass
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--start', default='2.0,4.5,90', help='x,y in metres and heading in degrees (90 = up the plan)')
    ap.add_argument('--drive-port', type=int, default=9769)
    ap.add_argument('--camera-port', type=int, default=9010)
    ap.add_argument('--exact-depth-port', type=int, default=9775)
    ap.add_argument('--no-camera', action='store_true', help='skip rendering (exact depth only; faster batch runs)')
    ap.add_argument('--speed-error', type=float, default=0.0, help='e.g. 0.1: drives 10%% faster than calibrated')
    ap.add_argument('--gyro-drift', type=float, default=0.02, help='spread of the gyro bias, deg/s')
    args = ap.parse_args()
    global SPEED_FACTOR, GYRO_BIAS_DPS
    SPEED_FACTOR, GYRO_BIAS_DPS = 1 + args.speed_error, args.gyro_drift
    sim = Sim([float(v) for v in args.start.split(',')], args.seed)
    state = drive_server.DriveState(SimTx(sim.rover), sim.imu)
    state.installed = 'rover-drive-0.9+sim c130 t50 s1200|0'
    for target in (sim.physics, state.watchdog) + (() if args.no_camera else (sim.camera,)):
        threading.Thread(target=target, daemon=True).start()
    cam = ThreadingHTTPServer(('127.0.0.1', args.camera_port), camera_handler(sim))
    threading.Thread(target=cam.serve_forever, daemon=True).start()
    exact = ThreadingHTTPServer(('127.0.0.1', args.exact_depth_port), exact_depth_handler(sim))
    threading.Thread(target=exact.serve_forever, daemon=True).start()
    drive = ThreadingHTTPServer(('127.0.0.1', args.drive_port), drive_server.make_handler(state))
    print(f'sim: drive http://127.0.0.1:{args.drive_port}  camera http://127.0.0.1:{args.camera_port}/live.jpg',
          flush=True)
    drive.serve_forever()


if __name__ == '__main__':
    main()
