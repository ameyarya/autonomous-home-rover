"""Measured moves and the odometer (roadmap item 1). Open loop: the rover has no wheel or
heading sensors yet, so distances and turns come from drive time and measured motion:

  forward_cm_s, reverse_cm_s   how fast it goes at the calibrated power (timed run + tape measure)
  turn_radius_cm               radius of the full-lock circle, from a short arc: R = (f² + s²) / 2s
                               (f forward, s sideways of where it started; a full circle won't fit indoors)
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
FIELDS = ('forward_cm_s', 'reverse_cm_s', 'turn_radius_cm', 'power', 'arc_forward_cm', 'arc_sideways_cm')
# arc_*: the raw short-arc measurement, kept so the Calibrate page can show what was entered
MAX_MOVE_S = 8.0
TEST_S = {'forward': 2.0, 'reverse': 1.5, 'arc': 2.0}   # timed runs for measuring speed and turning
# key -> (throttle: +1 forward / -1 reverse / 0, steer: +1 left / -1 right / 0)
# lower case = the fine steering heading hold uses (rover app 0.9): w/s wheels centred now,
# a/d/z/c steer GENTLE of full lock
GENTLE = 0.3
KEYS = {'W': (1, 0), 'A': (1, 1), 'D': (1, -1), 'S': (-1, 0), 'Z': (-1, 1), 'C': (-1, -1),
        'L': (0, 1), 'R': (0, -1),
        'w': (1, 0), 's': (-1, 0), 'a': (1, GENTLE), 'd': (1, -GENTLE), 'z': (-1, GENTLE), 'c': (-1, -GENTLE)}


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


def radius_from_arc(forward_cm, sideways_cm):
    """Full-lock radius from where a timed arc ended: forward along the start heading, sideways off it."""
    if sideways_cm <= 0 or forward_cm < 0:
        raise ValueError('measure forward and sideways (both positive)')
    return (forward_cm ** 2 + sideways_cm ** 2) / (2 * sideways_cm)


def arc_measured(m):
    """(degrees turned, cm travelled) in the TEST_S['arc'] run, from its forward/sideways offsets."""
    f, side = m.get('arc_forward_cm'), m.get('arc_sideways_cm')
    if not (f is not None and side):
        return None
    r = radius_from_arc(f, side)
    turned = 2 * math.asin(min(1.0, math.hypot(f, side) / (2 * r)))
    return math.degrees(turned), r * turned


def scaled(m, power):
    """Speeds scaled to the current power setting (roughly linear; re-measure after big changes).
    'arc' is the speed along a full-lock turn: from the arc run itself when measured (it times
    turns directly), else the forward speed."""
    f = power / m['power'] if m.get('power') and power else 1.0
    forward = (m.get('forward_cm_s') or 0) * f
    arc = arc_measured(m)
    return {'forward': forward, 'reverse': (m.get('reverse_cm_s') or 0) * f,
            'radius': m.get('turn_radius_cm') or 0, 'arc': arc[1] / TEST_S['arc'] * f if arc else forward}


def plan(kind, amount, m, power):
    """('forward'|'back', cm) or ('left'|'right', degrees) -> (key, seconds)."""
    s = scaled(m, power)
    if kind in ('forward', 'back'):
        speed = s['forward'] if kind == 'forward' else s['reverse']
        if not speed:
            raise ValueError(f'measure the {"forward" if kind == "forward" else "reverse"} speed first')
        key, seconds = ('W' if kind == 'forward' else 'S'), abs(amount) / speed
    elif kind in ('left', 'right'):
        if not (s['arc'] and s['radius']):
            raise ValueError('measure the turning circle (and forward speed) first')
        key, seconds = ('A' if kind == 'left' else 'D'), s['radius'] * math.radians(abs(amount)) / s['arc']
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

    def add(self, key, seconds, m, power, turn_deg=None):
        """turn_deg: the heading change the gyro measured over this step (else modelled from the key)."""
        if key not in KEYS or seconds <= 0:
            return
        throttle, steer = KEYS[key]
        s = scaled(m, power)
        speed = (s['arc'] if steer else s['forward']) if throttle > 0 else s['reverse'] if throttle < 0 else 0
        travel = throttle * speed * seconds                     # signed cm
        self.distance_cm += abs(travel)
        if turn_deg is not None:
            turn = math.radians(turn_deg)
        else:
            turn = travel / s['radius'] * steer if steer and s['radius'] else 0.0   # radians, + = left
        h0 = math.radians(self.heading_deg)
        if abs(turn) > 1e-6 and travel:                            # exact arc of radius travel/turn
            r = travel / turn
            self.x_cm += r * (math.sin(h0 + turn) - math.sin(h0))
            self.y_cm -= r * (math.cos(h0 + turn) - math.cos(h0))
        else:
            self.x_cm += travel * math.cos(h0)
            self.y_cm += travel * math.sin(h0)
        self.heading_deg = (self.heading_deg + math.degrees(turn) + 180) % 360 - 180

    def as_dict(self):
        return {'distance_cm': round(self.distance_cm, 1), 'x_cm': round(self.x_cm, 1), 'y_cm': round(self.y_cm, 1),
                'heading_deg': round(self.heading_deg, 1)}
