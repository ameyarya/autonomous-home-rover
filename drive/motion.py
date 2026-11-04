"""Measured moves and the odometer (roadmap item 1). Open loop: the rover has no wheel or
heading sensors yet, so distances and turns come from drive time and measured motion:

  forward_cm_s, reverse_cm_s   how fast it goes at the calibrated power (timed run + tape measure)
  turn_radius_cm               radius of the full-lock circle (drive a circle, measure its width)
  power                        the speed setting those were measured at (estimates scale with it)

A turn is a forward arc at full lock (car steering can't turn on the spot): X degrees takes
R * X(rad) of travel. Everything here is an estimate until roadmap items 3-6 add a gyro and
wheel odometry. Values live in drive/motion.json.
"""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
MOTION_FILE = HERE / 'motion.json'
FIELDS = ('forward_cm_s', 'reverse_cm_s', 'turn_radius_cm', 'power')
MAX_MOVE_S = 8.0
TEST_S = {'forward': 2.0, 'reverse': 1.5}        # timed runs for measuring speed
# key -> (throttle: +1 forward / -1 reverse / 0, steer: +1 left / -1 right / 0)
KEYS = {'W': (1, 0), 'A': (1, 1), 'D': (1, -1), 'S': (-1, 0), 'Z': (-1, 1), 'C': (-1, -1),
        'L': (0, 1), 'R': (0, -1)}


def load():
    try:
        values = json.loads(MOTION_FILE.read_text())
    except (OSError, ValueError):
        values = {}
    return {k: values.get(k) for k in FIELDS}


def validate(values):
    out = {}
    for k in FIELDS:
        v = values.get(k)
        if v is None:
            out[k] = None
        elif isinstance(v, (int, float)) and not isinstance(v, bool) and 0 < v < 10000:
            out[k] = round(float(v), 2)
        else:
            raise ValueError(f'{k} must be a positive number')
    return out


def save(values):
    values = validate(dict(load(), **values))
    MOTION_FILE.write_text(json.dumps(values, indent=1) + '\n')
    return values


def scaled(m, power):
    """Speeds scaled to the current power setting (roughly linear; re-measure after big changes)."""
    f = power / m['power'] if m.get('power') and power else 1.0
    return {'forward': (m.get('forward_cm_s') or 0) * f, 'reverse': (m.get('reverse_cm_s') or 0) * f,
            'radius': m.get('turn_radius_cm') or 0}


def plan(kind, amount, m, power):
    """('forward'|'back', cm) or ('left'|'right', degrees) -> (key, seconds)."""
    s = scaled(m, power)
    if kind in ('forward', 'back'):
        speed = s['forward'] if kind == 'forward' else s['reverse']
        if not speed:
            raise ValueError(f'measure the {"forward" if kind == "forward" else "reverse"} speed first')
        key, seconds = ('W' if kind == 'forward' else 'S'), abs(amount) / speed
    elif kind in ('left', 'right'):
        if not (s['forward'] and s['radius']):
            raise ValueError('measure the forward speed and the turning circle first')
        key, seconds = ('A' if kind == 'left' else 'D'), s['radius'] * math.radians(abs(amount)) / s['forward']
    else:
        raise ValueError('kind must be forward, back, left or right')
    if seconds > MAX_MOVE_S:
        raise ValueError(f'that move would take {seconds:.1f} s; the limit is {MAX_MOVE_S:.0f} s')
    return key, seconds


class Odometer:
    """Dead-reckoned distance and pose from drive time (estimate; drifts)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.distance_cm = 0.0
        self.x_cm = self.y_cm = 0.0
        self.heading_deg = 0.0

    def add(self, key, seconds, m, power):
        if key not in KEYS or seconds <= 0:
            return
        throttle, steer = KEYS[key]
        s = scaled(m, power)
        speed = s['forward'] if throttle > 0 else s['reverse'] if throttle < 0 else 0
        travel = throttle * speed * seconds                     # signed cm
        self.distance_cm += abs(travel)
        turn = math.degrees(travel / s['radius']) * steer if steer and s['radius'] else 0.0
        mid = math.radians(self.heading_deg + turn / 2)
        self.x_cm += travel * math.cos(mid)
        self.y_cm += travel * math.sin(mid)
        self.heading_deg = (self.heading_deg + turn + 180) % 360 - 180

    def as_dict(self):
        return {'distance_cm': round(self.distance_cm, 1), 'x_cm': round(self.x_cm, 1), 'y_cm': round(self.y_cm, 1),
                'heading_deg': round(self.heading_deg, 1)}
