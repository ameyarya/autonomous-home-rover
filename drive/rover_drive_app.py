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

Calibrate on the bench with the wheels off the ground: motor signs, steering channel, centre
and travel. Start with small STEER_TRAVEL; a servo forced past the linkage's end stalls.
"""
import time
import __main__ as receiver

# Calibration: the drive server rewrites these lines from drive/calibration.json when you press
# "Install on rover" on the dashboard's Calibrate page (http://rover.local:8010/calibrate).
VERSION = 'rover-drive-0.4'
SPEED = 900                 # of 2048; indoor dog-following pace
MOTOR_SIGNS = (1, 1)        # flip one if that axle drives backwards
STEER_SIGN = 1              # -1 swaps left/right if A turns the wheels right
STEER_CHANNEL = 2           # receiver servo port S2
STEER_CENTER = 90
STEER_TRAVEL = 20           # degrees each way; raise only after checking the linkage limits
STEER_HOLD_MS = 500         # steering re-centres if no A/D arrives
MIX_MS = 400                # a throttle key and a steer key this close together combine


class TankApp:
    def __init__(self):
        self.version = VERSION
        self.servos = None
        self.steer = 0
        self.steer_deadline = time.ticks_ms()
        a, b = MOTOR_SIGNS
        self.throttles = {b'W': (a * SPEED, b * SPEED), b'S': (-a * SPEED, -b * SPEED)}
        self.steers = {b'A': -1, b'D': 1}
        self.throttle, self.throttle_at = (0, 0), None
        self.wanted, self.wanted_at = 0, None

    def fresh(self, at, now):
        return at is not None and time.ticks_diff(now, at) < MIX_MS

    def command(self, key):
        now = time.ticks_ms()
        if key in self.throttles:
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
            if key in self.steers:
                self.steer_deadline = time.ticks_add(now, STEER_HOLD_MS)
        return motors

    def set_steer(self, steer):
        self.steer = steer
        self.servos.set_angle(STEER_CHANNEL, STEER_CENTER + STEER_SIGN * steer * STEER_TRAVEL)

    def ensure_servo(self):
        if self.servos is not None:
            return
        from servos import ServosController
        from machine import Timer
        self.servos = ServosController()
        self.servos.set_angle(STEER_CHANNEL, STEER_CENTER)

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
