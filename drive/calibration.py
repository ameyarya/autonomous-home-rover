"""Rover drive calibration: drive/calibration.json -> the constants at the top of
rover_drive_app.py, installed on the receiver with the signed app update.

The receiver only takes W/A/S/D/X; motor directions and the servo port are app constants (a new
signed install each). Centre, range and speed are also calibrated live (app 0.6 codes from the Calibrate page: saved on
the receiver, reported in the version string); `sync_from_rover` copies them back here.
"""
import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAL_FILE = HERE / 'calibration.json'
APP_FILE = HERE / 'rover_drive_app.py'
BASE_VERSION = 'rover-drive-0.6'

LIMITS = {
    'speed': (200, 1600),          # of 2048
    'steer_channel': (1, 4),
    'steer_center': (45, 135),     # degrees
    'steer_travel': (0, 90),       # degrees each way; centre ± travel must stay within 0..180
}


def validate(values):
    """Return a clean calibration dict or raise ValueError."""
    out = {}
    for key, (lo, hi) in LIMITS.items():
        v = values.get(key)
        if not isinstance(v, (int, float)) or isinstance(v, bool) or not lo <= v <= hi:
            raise ValueError(f'{key} must be a number {lo}..{hi}')
        out[key] = int(round(v))
    signs = values.get('motor_signs')
    if not (isinstance(signs, (list, tuple)) and len(signs) == 2 and all(s in (1, -1) for s in signs)):
        raise ValueError('motor_signs must be two of 1 / -1')
    out['motor_signs'] = [int(s) for s in signs]
    if values.get('steer_sign') not in (1, -1):
        raise ValueError('steer_sign must be 1 or -1')
    out['steer_sign'] = int(values['steer_sign'])
    if out['steer_center'] - out['steer_travel'] < 0 or out['steer_center'] + out['steer_travel'] > 180:
        raise ValueError('steer_center ± steer_travel must stay within 0..180 degrees')
    return out


def load():
    return validate(json.loads(CAL_FILE.read_text()))


def save(values):
    values = validate(values)
    CAL_FILE.write_text(json.dumps(values, indent=1) + '\n')
    return values


def version(values):
    digest = hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()[:6]
    return f'{BASE_VERSION}+c{digest}'


def render(values, source=None):
    """App source (bytes) with the calibration constants replaced."""
    values = validate(values)
    text = APP_FILE.read_text() if source is None else source
    lines = {
        'VERSION': repr(version(values)),
        'SPEED': str(values['speed']),
        'MOTOR_SIGNS': '({}, {})'.format(*values['motor_signs']),
        'STEER_SIGN': str(values['steer_sign']),
        'STEER_CHANNEL': str(values['steer_channel']),
        'STEER_CENTER': str(values['steer_center']),
        'STEER_TRAVEL': str(values['steer_travel']),
    }
    for name, value in lines.items():
        text, n = re.subn(rf'^{name} = [^#\n]*?(\s*(#.*)?)$', lambda m: f'{name} = {value}{m.group(1)}', text,
                          count=1, flags=re.M)
        if n != 1:
            raise ValueError(f'{name} not found in the app source')
    compile(text, 'rover_drive_app.py', 'exec')
    return text.encode()


def live_values(installed):
    """Centre/range/speed from the receiver's version ('... c92 t45 s900[*]|0'), or None.
    'unsaved' is True while remote edits aren't saved on the rover yet."""
    m = re.search(r' c(\d+) t(\d+) s(\d+)(\*?)(\||$)', installed or '')
    if not m:
        return None
    return {'steer_center': int(m.group(1)), 'steer_travel': int(m.group(2)), 'speed': int(m.group(3)),
            'unsaved': bool(m.group(4))}


def sync_from_rover(installed):
    """Save the rover's saved live calibration into calibration.json; returns True if it changed."""
    live = live_values(installed)
    if not live or live['unsaved']:
        return False
    values = load()
    new = dict(values, **{k: live[k] for k in ('steer_center', 'steer_travel', 'speed')})
    if new == values:
        return False
    save(new)
    return True


LIVE_CODES = {'steer_center': ('WW', 'WS', 1), 'steer_travel': ('AW', 'AS', 1), 'speed': ('DW', 'DS', 10)}


def codes_to(current, target):
    """Calibration codes that move the rover's live values `current` to `target` (same keys as
    live_values)."""
    codes = []
    for key, (up, down, step) in LIVE_CODES.items():
        if key in target:
            n = round((target[key] - current[key]) / step)
            codes += [up] * n if n > 0 else [down] * -n
    return codes
