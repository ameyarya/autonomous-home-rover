"""Rover + dog world for closed-loop tests of the real behaviour code.

Kinematic models posed into a MuJoCo scene (sim/rover.xml); MuJoCo renders the rover camera and
gives ground-truth dog pixels through segmentation. Not physics: the rover is a kinematic bicycle
(front-wheel steering) and the dog follows a script. Units: metres, seconds, radians.

All rover numbers are estimates until measured on the assembled chassis (see RoverParams).
"""
import math
import random
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).with_name('rover.xml')
W, H = 320, 240        # render size; detections are in percent of the image like the watcher's


@dataclass
class RoverParams:
    wheelbase: float = 0.157       # front-rear axle distance, from the chassis STEP
    length: float = 0.23           # footprint, OpenFrame Two spec
    width: float = 0.18
    rear_overhang: float = 0.035   # rear axle to rear edge (estimate)
    steer_deg: float = 20.0        # front-wheel angle at full A/D (assumed ~ servo travel 20 deg)
    speed: float = 0.30            # m/s at SPEED 900 of 2048 (estimate; measure on the chassis)
    accel_tau: float = 0.08        # s, first-order speed response
    stop_latency: float = 0.05     # s of motion after the pulse ends (X delivery + coast)
    ack_s: float = 0.30            # s until the drive server returns (stop ACK), measured on the bench


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float
    name: str


# Obstacles in 2D (walls as thin rects), matching rover.xml.
OBSTACLES = [
    Rect(-0.1, -0.1, 5.0, 0.0, 'wall'), Rect(-0.1, 4.0, 5.0, 4.1, 'wall'), Rect(-0.1, -0.1, 0.0, 4.1, 'wall'),
    Rect(5.0, -0.1, 5.1, 1.6, 'wall'), Rect(5.0, 2.4, 5.1, 4.1, 'wall'),
    Rect(5.0, 1.1, 7.1, 1.2, 'wall'), Rect(5.0, 2.8, 7.1, 2.9, 'wall'), Rect(7.0, 1.1, 7.1, 2.9, 'wall'),
    Rect(1.0, 3.25, 3.0, 4.0, 'sofa'), Rect(3.8, 0.3, 4.6, 1.0, 'table'),
]


def rover_corners(x, y, yaw, p):
    """Footprint corners; (x, y) is the rear-axle centre, yaw the heading."""
    c, s = math.cos(yaw), math.sin(yaw)
    back, front, half = -p.rear_overhang, p.length - p.rear_overhang, p.width / 2
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((back, -half), (front, -half), (front, half), (back, half))]


def _poly_rect_overlap(poly, r):
    xs, ys = [q[0] for q in poly], [q[1] for q in poly]
    if max(xs) < r.x0 or min(xs) > r.x1 or max(ys) < r.y0 or min(ys) > r.y1:
        return False
    rect = [(r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1)]
    for shape in (poly, rect):
        for i in range(4):
            (ax, ay), (bx, by) = shape[i], shape[(i + 1) % 4]
            nx, ny = ay - by, bx - ax
            pa = [nx * px + ny * py for px, py in poly]
            pb = [nx * px + ny * py for px, py in rect]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def _point_poly_distance(px, py, poly):
    inside = True
    best = float('inf')
    for i in range(4):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % 4]
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) < 0:
            inside = False
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(px - ax - t * dx, py - ay - t * dy))
    return 0.0 if inside else best


class Dog:
    """Scripted dog: a queue of tasks. Body approximated by three circles for collisions."""
    RADIUS = 0.12

    def __init__(self, x, y, yaw, script, seed=0):
        self.x, self.y, self.yaw = x, y, yaw
        self.script = list(script)
        self.task = None
        self.task_t = 0.0
        self.posture = 'stand'       # stand | lie
        self.asleep = False
        self.wag = 0.0               # tail angle
        self.head = 0.0              # head bob offset
        self.speed = 0.0
        self.gait = 0.0              # walking phase, radians
        self.rng = random.Random(seed)

    def circles(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * a, self.y + s * a) for a in (-0.18, 0.0, 0.2)]

    def activity(self):
        if self.asleep:
            return 'sleeping'
        if self.posture == 'lie':
            return 'resting'
        return 'active'

    def step(self, dt, t, blocked):
        """Advance dt seconds. `blocked(x, y, yaw)` says whether a pose would hit the rover/room."""
        self.speed = 0.0
        if self.task is None:
            if not self.script:
                self.task = ('stand', 1e9, False)
            else:
                self.task = self.script.pop(0)
            self.task_t = 0.0
        kind = self.task[0]
        self.task_t += dt
        self.wag = 0.0
        if kind in ('walk', 'run'):
            _, (tx, ty), speed = self.task
            self.posture, self.asleep = 'stand', False
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self.task = None
                return
            want = math.atan2(dy, dx)
            turn = (want - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += max(-4 * dt, min(4 * dt, turn))
            if abs(turn) < 0.6:
                step = min(dist, speed * dt)
                nx, ny = self.x + math.cos(self.yaw) * step, self.y + math.sin(self.yaw) * step
                if not blocked(nx, ny, self.yaw):
                    self.x, self.y, self.speed = nx, ny, speed
                    self.gait += dt * 2 * math.pi * (1.5 + speed)     # ~2 strides/s walking, faster running
            self.wag = 0.5 * math.sin(t * 12)
        elif kind == 'approach':            # walk/run toward a moving target until stop distance
            _, target_fn, speed, stop = self.task
            self.posture, self.asleep = 'stand', False
            tx, ty = target_fn()
            dx, dy = tx - self.x, ty - self.y
            self.yaw = math.atan2(dy, dx)
            if math.hypot(dx, dy) - 0.2 <= stop or self.task_t > 8:
                self.task = None
                return
            step = speed * dt
            nx, ny = self.x + math.cos(self.yaw) * step, self.y + math.sin(self.yaw) * step
            if not blocked(nx, ny, self.yaw):
                self.x, self.y, self.speed = nx, ny, speed
                self.gait += dt * 2 * math.pi * (1.5 + speed)
        elif kind == 'stand':
            _, secs, wag = self.task
            self.posture, self.asleep = 'stand', False
            if wag:
                self.wag = 0.6 * math.sin(t * 18)
            if self.task_t >= secs:
                self.task = None
        elif kind == 'lie':
            _, secs = self.task
            self.posture, self.asleep = 'lie', False
            if self.rng.random() < dt / 6:       # occasional head movement while awake
                self.head = 0.04 if self.head == 0 else 0.0
            if self.task_t >= secs:
                self.task = None
        elif kind == 'sleep':
            _, secs = self.task
            self.posture, self.asleep, self.head = 'lie', True, 0.0
            if self.task_t >= secs:
                self.task = None
        elif kind == 'turn':
            _, yaw = self.task
            self.yaw = yaw
            self.task = None


class World:
    def __init__(self, params=None, seed=0, render=True):
        self.p = params or RoverParams()
        self.model = mujoco.MjModel.from_xml_path(str(XML))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=H, width=W) if render else None
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self.dog_geoms = {i for i in range(self.model.ngeom) if (self.model.geom(i).name or '').startswith('dog_')}
        self.base_pos = {i: self.model.geom_pos[i].copy() for i in self.dog_geoms}
        self.base_quat = {i: self.model.geom_quat[i].copy() for i in self.dog_geoms}
        self.t = 0.0
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.dog = None
        self.dog_contacts = 0       # rover moves stopped by the dog
        self.bumps = 0              # rover moves stopped by walls/furniture
        self.odometer = 0.0
        self.last_pulse_blocked = False

    # ---- setup ----
    def place(self, rover, dog):
        self.x, self.y, self.yaw = rover
        self.dog = dog
        dog.step(0.0, self.t, lambda *a: True)   # take the first task's posture before frame 0
        self._pose()

    # ---- geometry ----
    def rover_hits(self, x, y, yaw):
        poly = rover_corners(x, y, yaw, self.p)
        for r in OBSTACLES:
            if _poly_rect_overlap(poly, r):
                return r.name
        if self.dog and any(_point_poly_distance(cx, cy, poly) < Dog.RADIUS for cx, cy in self.dog.circles()):
            return 'dog'
        return None

    def dog_blocked(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        circles = [(x + c * a, y + s * a) for a in (-0.18, 0.0, 0.2)]
        poly = rover_corners(self.x, self.y, self.yaw, self.p)
        if any(_point_poly_distance(cx, cy, poly) < Dog.RADIUS + 0.03 for cx, cy in circles):
            return True
        for r in OBSTACLES:
            for cx, cy in circles:
                if r.x0 - Dog.RADIUS < cx < r.x1 + Dog.RADIUS and r.y0 - Dog.RADIUS < cy < r.y1 + Dog.RADIUS:
                    return True
        return False

    def gap_to_dog(self):
        poly = rover_corners(self.x, self.y, self.yaw, self.p)
        return min(_point_poly_distance(cx, cy, poly) for cx, cy in self.dog.circles()) - Dog.RADIUS

    # ---- time ----
    def advance(self, seconds, key=None):
        """Advance the world; `key` (W/A/S/D) drives the rover meanwhile."""
        dt = 0.01
        steps = max(1, int(round(seconds / dt)))
        p = self.p
        for _ in range(steps):
            target = 0.0
            steer = 0.0
            if key in ('W', 'A', 'D'):
                target = p.speed
                steer = {'W': 0.0, 'A': 1.0, 'D': -1.0}[key] * math.radians(p.steer_deg)
            elif key == 'S':
                target = -p.speed
            self.v += (target - self.v) * min(1.0, dt / p.accel_tau)
            if abs(self.v) > 1e-4:
                yaw = self.yaw + self.v / p.wheelbase * math.tan(steer) * dt
                x = self.x + self.v * math.cos(self.yaw) * dt
                y = self.y + self.v * math.sin(self.yaw) * dt
                hit = self.rover_hits(x, y, yaw)
                if hit:
                    if hit == 'dog':
                        self.dog_contacts += 1
                    else:
                        self.bumps += 1
                    self.v = 0.0
                    key = None
                else:
                    self.odometer += abs(self.v) * dt
                    self.x, self.y, self.yaw = x, y, yaw
            self.dog.step(dt, self.t, self.dog_blocked)
            self.t += dt
        self._pose()

    def pulse(self, key, ms):
        """One drive-server pulse: motion for ms (+ stop latency), then the time until the ACK."""
        before = self.dog_contacts + self.bumps
        start = (self.x, self.y)
        self.advance(ms / 1000 + self.p.stop_latency, key)
        moved = math.hypot(self.x - start[0], self.y - start[1])
        self.last_pulse_blocked = self.dog_contacts + self.bumps > before and moved < 0.02
        self.v = 0.0
        self.advance(max(0.0, self.p.ack_s - self.p.stop_latency))

    # ---- rendering / perception ----
    def _pose(self):
        d = self.data
        d.mocap_pos[0] = [self.x + self.p.wheelbase / 2 * math.cos(self.yaw), self.y + self.p.wheelbase / 2 * math.sin(self.yaw), 0]
        d.mocap_quat[0] = [math.cos(self.yaw / 2), 0, 0, math.sin(self.yaw / 2)]
        dog = self.dog
        if dog is None:
            return
        d.mocap_pos[1] = [dog.x, dog.y, 0]
        d.mocap_quat[1] = [math.cos(dog.yaw / 2), 0, 0, math.sin(dog.yaw / 2)]
        m = self.model
        for i in self.dog_geoms:
            name = m.geom(i).name
            pos = self.base_pos[i].copy()
            if dog.posture == 'lie':
                pos[2] -= 0.21 if 'leg' not in name else 0.13
                if name in ('dog_head', 'dog_snout', 'dog_ear_l', 'dog_ear_r'):
                    pos[2] += 0.06 - (0.08 if dog.asleep else 0) + dog.head
            if dog.speed > 0 and dog.posture == 'stand':
                pos[2] += 0.012 * math.sin(2 * dog.gait) if 'leg' not in name else 0.0   # body bob
            m.geom_pos[i] = pos
            if 'leg' in name:
                # Diagonal pairs swing in opposite phase while walking.
                phase = 0.0 if name in ('dog_leg_fl', 'dog_leg_rr') else math.pi
                a = 0.4 * math.sin(dog.gait + phase) if dog.speed > 0 and dog.posture == 'stand' else 0.0
                m.geom_quat[i] = [math.cos(a / 2), 0, math.sin(a / 2), 0]
            if name == 'dog_tail':
                a = dog.wag
                q = np.array([math.cos(a / 2), 0, 0, math.sin(a / 2)])
                out = np.zeros(4)
                mujoco.mju_mulQuat(out, q, self.base_quat[i])
                m.geom_quat[i] = out
        mujoco.mj_forward(m, d)

    def camera(self):
        """Render the rover camera: (gray 160x120 float with sensor noise, dog mask)."""
        r = self.renderer
        r.update_scene(self.data, camera='rover_cam')
        rgb = r.render().astype(np.float32)
        r.enable_segmentation_rendering()
        try:
            r.update_scene(self.data, camera='rover_cam')
            seg = r.render()
        finally:
            r.disable_segmentation_rendering()
        mask = np.isin(seg[..., 0], list(self.dog_geoms)) & (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM)
        gray = rgb @ np.array([0.299, 0.587, 0.114], np.float32)
        gray += self.np_rng.normal(0, 2.0, gray.shape).astype(np.float32)
        gray = gray.reshape(120, 2, 160, 2).mean(axis=(1, 3))
        return gray, mask, rgb

    def detect(self, mask):
        """Simulated YOLO: box from the dog's visible pixels, with misses and jitter."""
        ys, xs = np.nonzero(mask)
        if len(xs) < 40:
            return None
        x1, x2, y1, y2 = xs.min() / W, (xs.max() + 1) / W, ys.min() / H, (ys.max() + 1) / H
        truncated = x1 <= 0.0 or x2 >= 1.0
        height = y2 - y1
        miss = 0.04 + (0.25 if height < 0.10 else 0) + (0.2 if truncated else 0) + (0.1 if self.dog.posture == 'lie' else 0)
        if self.rng.random() < miss:
            return None
        j = lambda: self.rng.uniform(-0.015, 0.015)
        x1, x2 = max(0, x1 + j() * (x2 - x1)), min(1, x2 + j() * (x2 - x1))
        y1, y2 = max(0, y1 + j() * height), min(1, y2 + j() * height)
        return {'box': (x1, y1, x2, y2), 'confidence': round(self.rng.uniform(0.4, 0.85), 3)}

    def close(self):
        if self.renderer:
            self.renderer.close()
