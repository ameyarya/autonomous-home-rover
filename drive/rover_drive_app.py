"""OpenFrame Two drive app for the CyberBrick receiver's wireless service.

Installed over the air (signed) into the service the Mini-T project put on the receiver; the
service keeps its 500 ms hold-to-drive motor watchdog, X+ACK stop, and rollback. The class name
must stay TankApp (the service loads it by that name).

  W  forward                          A  steer left
  S  reverse                          D  steer right
  X  stop, steering centred

Throttle and steering combine: a W/S and an A/D less than MIX_MS apart drive and steer together
(the sender repeats both), A/D alone only turn the servo, W/S alone drive with the wheels centred.
The Mac sends W with A/D for its forward arcs.

Live calibration (0.5): the receiver only passes these five keys, so the remote sends edits as a
code: D A D A back-to-back (each under CODE_GAP_MS apart, which driving never produces) and two
code keys. Code keys never drive. Codes (see CODES): centre ±1°, range ±5°, speed ±50, save,
revert. Saved values go to CAL_FILE on the receiver and win over the constants below; the
current values are in the version string ("c92 t45 s900", "*" = unsaved), which the Mac reads.
"""
import time
import __main__ as receiver

# Calibration: the drive server rewrites these lines from drive/calibration.json when you press
# "Install on rover" on the dashboard's Calibrate page (http://rover.local:8010/calibrate).
VERSION = 'rover-drive-0.5'
SPEED = 900                 # of 2048; indoor dog-following pace
MOTOR_SIGNS = (1, 1)        # flip one if that axle drives backwards
STEER_SIGN = 1              # -1 swaps left/right if A turns the wheels right
STEER_CHANNEL = 2           # receiver servo port S2
STEER_CENTER = 90
STEER_TRAVEL = 20           # degrees each way; raise only after checking the linkage limits
STEER_HOLD_MS = 500         # steering re-centres if no A/D arrives
MIX_MS = 400                # a throttle key and a steer key this close together combine
CODE_GAP_MS = 30            # calibration code keys arrive back-to-back
CAL_FILE = 'rover_cal.json'
CODES = {b'WW': ('center', 1), b'WS': ('center', -1), b'AW': ('travel', 5), b'AS': ('travel', -5),
         b'DW': ('speed', 50), b'DS': ('speed', -50), b'SS': ('save', 0), b'SA': ('revert', 0)}
LIMITS = {'center': (45, 135), 'travel': (0, 90), 'speed': (200, 1600)}


class TankApp:
    def __init__(self):
        self.servos = None
        self.steer = 0
        self.steer_deadline = time.ticks_ms()
        self.throttle, self.throttle_at = (0, 0), None
        self.wanted, self.wanted_at = 0, None
        self.recent = []            # (key, ms) of the last keys, for calibration codes
        self.code = None            # code keys collected after the D A D A prefix
        self.cal = self.load_cal()
        self.dirty = False
        self.apply_cal()

    def load_cal(self):
        cal = {'center': STEER_CENTER, 'travel': STEER_TRAVEL, 'speed': SPEED}
        try:
            import json
            with open(CAL_FILE) as f:
                saved = json.load(f)
            for k in cal:
                if isinstance(saved.get(k), int):
                    cal[k] = saved[k]
        except Exception:
            pass
        return self.clamp(cal)

    def clamp(self, cal):
        for k, (lo, hi) in LIMITS.items():
            cal[k] = max(lo, min(hi, cal[k]))
        cal['travel'] = min(cal['travel'], cal['center'], 180 - cal['center'])
        return cal

    def apply_cal(self):
        a, b = MOTOR_SIGNS
        speed = self.cal['speed']
        self.throttles = {b'W': (a * speed, b * speed), b'S': (-a * speed, -b * speed)}
        self.steers = {b'A': -1, b'D': 1}
        self.version = '%s c%d t%d s%d%s' % (VERSION, self.cal['center'], self.cal['travel'], speed,
                                             '*' if self.dirty else '')

    def calibrate(self, code):
        what, step = CODES[code]
        if what == 'save':
            import json
            with open(CAL_FILE, 'w') as f:
                json.dump(self.cal, f)
            self.dirty = False
        elif what == 'revert':
            self.cal, self.dirty = self.load_cal(), False
        else:
            self.cal[what] += step
            self.cal = self.clamp(self.cal)
            self.dirty = True
        self.apply_cal()

    def fresh(self, at, now):
        return at is not None and time.ticks_diff(now, at) < MIX_MS

    def take_code_key(self, key, now):
        """True if `key` belongs to a calibration code (then it must not drive)."""
        last = self.recent[-1][1] if self.recent else None
        self.recent = (self.recent + [(key, now)])[-4:]
        quick = last is not None and time.ticks_diff(now, last) < CODE_GAP_MS
        if self.code is not None:
            if not quick:
                self.code = None
                return False
            self.code += key
            if len(self.code) == 2:
                code, self.code = self.code, None
                if code in CODES:
                    self.calibrate(code)
            return True
        keys = b''.join(k for k, _ in self.recent)
        if keys == b'DADA' and time.ticks_diff(now, self.recent[0][1]) < 3 * CODE_GAP_MS:
            self.code = b''
        return False

    def command(self, key):
        now = time.ticks_ms()
        coded = self.take_code_key(key, now)
        if coded:
            motors, steer = (0, 0), 0
        elif key in self.throttles:
            self.throttle, self.throttle_at = self.throttles[key], now
            motors = self.throttle
            steer = self.wanted if self.fresh(self.wanted_at, now) else 0
        elif key in self.steers:
            self.wanted, self.wanted_at = self.steers[key], now
            motors = self.throttle if self.fresh(self.throttle_at, now) else (0, 0)
            steer = self.wanted
        else:
            self.throttle_at = self.wanted_at = None
            motors, steer = (0, 0), 0
        # The service validates candidate apps by calling command(); only touch the servo
        # once this is the active, confirmed app.
        if getattr(receiver, 'app', None) is self and getattr(receiver, 'candidate_id', None) is None:
            self.ensure_servo()
            self.set_steer(steer)
            if key in self.steers and not coded:
                self.steer_deadline = time.ticks_add(now, STEER_HOLD_MS)
        return motors

    def set_steer(self, steer):
        self.steer = steer
        self.servos.set_angle(STEER_CHANNEL, self.cal['center'] + STEER_SIGN * steer * self.cal['travel'])

    def ensure_servo(self):
        if self.servos is not None:
            return
        from servos import ServosController
        from machine import Timer
        self.servos = ServosController()
        self.servos.set_angle(STEER_CHANNEL, self.cal['center'])

        def steer_watchdog(timer):
            active = receiver.app
            check = getattr(active, 'check_steer_timeout', None)
            if check is not None:
                check()

        receiver._aux_timer = Timer(1)
        receiver._aux_timer.init(period=50, mode=Timer.PERIODIC, callback=steer_watchdog)

    def check_steer_timeout(self):
        if self.steer and time.ticks_diff(time.ticks_ms(), self.steer_deadline) >= 0:
            self.set_steer(0)

    def reset_aux(self):
        if self.servos is not None and self.steer:
            self.set_steer(0)
