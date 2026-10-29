"""Rover drive calibration: drive/calibration.json -> the constants at the top of
rover_drive_app.py, installed on the receiver with the signed app update.

The receiver only takes W/A/S/D/X, so calibration can't be sent live: each change is a new app
version (VERSION gets a short hash of the values, so the installed version shows what's on it).
"""
import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAL_FILE = HERE / 'calibration.json'
APP_FILE = HERE / 'rover_drive_app.py'
BASE_VERSION = 'rover-drive-0.3'

LIMITS = {
    'speed': (200, 1600),          # of 2048
    'steer_channel': (1, 4),
    'steer_center': (45, 135),     # degrees
    'steer_travel': (0, 45),       # degrees each way
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
    steer_only = values.get('steer_only', False)
    if not isinstance(steer_only, bool):
        raise ValueError('steer_only must be true or false')
    out['steer_only'] = steer_only
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
    hashed = {k: v for k, v in values.items() if k != 'steer_only'}   # steer-only shows as the suffix
    digest = hashlib.sha256(json.dumps(hashed, sort_keys=True).encode()).hexdigest()[:6]
    return f'{BASE_VERSION}+c{digest}' + ('-steertest' if values.get('steer_only') else '')


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
        'STEER_ONLY': 'True' if values['steer_only'] else 'False',
    }
    for name, value in lines.items():
        text, n = re.subn(rf'^{name} = [^#\n]*?(\s*(#.*)?)$', lambda m: f'{name} = {value}{m.group(1)}', text,
                          count=1, flags=re.M)
        if n != 1:
            raise ValueError(f'{name} not found in the app source')
    compile(text, 'rover_drive_app.py', 'exec')
    return text.encode()
