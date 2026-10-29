"""OpenFrame Two drive app for the CyberBrick receiver's wireless service.

Installed over the air (signed) into the service the Mini-T project put on the receiver; the
service keeps its 500 ms hold-to-drive motor watchdog, X+ACK stop, and rollback. The class name
must stay TankApp (the service loads it by that name).

  W  forward, steering centred        A  forward + steer left
  S  reverse, steering centred        D  forward + steer right
  X  stop, steering centred

Calibrate on the bench with the wheels off the ground: motor signs, steering channel, centre
and travel. Start with small STEER_TRAVEL; a servo forced past the linkage's end stalls.
"""
import time
import __main__ as receiver

# Calibration: the drive server rewrites these lines from drive/calibration.json when you press
# "Install on rover" on the dashboard's Calibrate page (http://rover.local:8010/calibrate).
VERSION = 'rover-drive-0.3'
SPEED = 900                 # of 2048; indoor dog-following pace
MOTOR_SIGNS = (1, 1)        # flip one if that axle drives backwards
STEER_SIGN = 1              # -1 swaps left/right if A turns the wheels right
STEER_CHANNEL = 2           # receiver servo port S2
STEER_CENTER = 90
STEER_TRAVEL = 20           # degrees each way; raise only after checking the linkage limits
STEER_ONLY = False          # bench test: A/D turn the servo without driving (never leave on)
STEER_HOLD_MS = 500         # steering re-centres if no command arrives


class TankApp:
    def __init__(self):
        self.version = VERSION
        self.servos = None
        self.steer = 0
        self.steer_deadline = time.ticks_ms()
        a, b = MOTOR_SIGNS
        fwd = (a * SPEED, b * SPEED)
        rev = (-a * SPEED, -b * SPEED)
        arc = (0, 0) if STEER_ONLY else fwd
        self.moves = {b'W': (fwd, 0), b'A': (arc, -1), b'D': (arc, 1), b'S': (rev, 0), b'X': ((0, 0), 0)}

    def command(self, key):
        motors, steer = self.moves.get(key, ((0, 0), 0))
        # The service validates candidate apps by calling command(); only touch the servo
        # once this is the active, confirmed app.
        if getattr(receiver, 'app', None) is self and getattr(receiver, 'candidate_id', None) is None:
            self.ensure_servo()
            self.set_steer(steer)
            self.steer_deadline = time.ticks_add(time.ticks_ms(), STEER_HOLD_MS)
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
