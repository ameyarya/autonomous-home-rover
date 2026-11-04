"""Heading from the M5StickS3 on the rover (firmware/m5stick-imu), over UDP port 8773.

The stick streams {"yaw", "rate", "still", "cal", "bumps", "bat", ...} ~50 times a second to
whoever sent it "sub" in the last 3 s; yaw is unwrapped degrees, + = turned left.

Heading hold: while driving straight, steer toward the heading the move started with.
The steering keys are all-or-nothing (full lock), so the error is predicted LEAD_S ahead from
the turn rate to stop before overshooting, small errors inside DEADBAND_DEG are left alone, and
corrections use the rover app's fine steering: gentle steer (a third of full lock) up to FULL_DEG,
full lock only beyond, and "wheels centred now" when on course (a lone W would keep the last steer
for 400 ms, so every tap used to last 400 ms).
Road runs 2026-11-03: full lock every step zigzagged ±4°; short full-lock taps still felt zig-zaggy;
gentle-only steered left every step for 1.7 s and still drifted right 4° (the front-wheel play
eats a 15° servo move). So HeadingHold works out a steering amount (proportional to the predicted
error, plus a slowly built trim for a steady pull) and mixes gentle and full-lock steps to get it.
"""
import json
import socket
import threading
import time

PORT = 8773
STALE_S = 0.3            # no packet for this long = offline
SUB_EVERY_S = 1.0
DEADBAND_DEG = 2.0
LEAD_S = 0.4             # servo + radio + Wi-Fi delay, plus the rover's own response
FULL_DEG = 10.0          # predicted error beyond which it uses full lock
TURN_LEAD_S = 0.15       # turns: 0.4 s stopped a 90° turn 6° early, it only coasted ~2°
KP = 1 / 10.0            # steering amount per degree of predicted error (10° off = full lock)
KI = 0.08                # trim built per degree-second of error
TRIM_MAX = 0.6
RATE_SMOOTH = 0.5        # the wobble spikes the turn rate; smooth it a little
STALL_S = 1.0            # a gyro turn with no heading change for this long = stuck
STALL_DPS = 2.0


class HeadingHold:
    """Steering amount u in [-1, 1] (+ = left) from the error, made from gentle (a/d) and full-lock
    (A/D) steps: |u| <= GENTLE mixes centred and gentle steps, above that gentle and full lock."""
    def __init__(self, throttle, gentle=0.3):
        self.throttle, self.gentle = throttle, gentle
        self.trim, self.rate, self.acc = 0.0, 0.0, 0.0

    def amount(self, error_deg, rate_dps, dt):
        self.rate += RATE_SMOOTH * (rate_dps - self.rate)
        predicted = error_deg - self.rate * LEAD_S
        self.trim = max(-TRIM_MAX, min(TRIM_MAX, self.trim + KI * error_deg * dt))
        u = KP * predicted + self.trim
        if abs(predicted) <= DEADBAND_DEG and abs(self.trim) < 0.05:
            u = 0.0
        return max(-1.0, min(1.0, u))

    def key(self, error_deg, rate_dps, dt):
        u = self.amount(error_deg, rate_dps, dt)
        left = u > 0
        fwd = self.throttle > 0
        straight, gentle = ('w' if fwd else 's'), (('a' if left else 'd') if fwd else ('c' if left else 'z'))
        mag = abs(u)
        if mag <= self.gentle:                       # centred or gentle, in proportion
            low, high, share = straight, gentle, mag / self.gentle
        else:                                        # gentle or full lock, in proportion
            low, high, share = gentle, gentle.upper(), (mag - self.gentle) / (1 - self.gentle)
        self.acc += share
        if self.acc >= 1.0:
            self.acc -= 1.0
            return high
        return low


def wrap(deg):
    return (deg + 180) % 360 - 180


class Imu:
    def __init__(self, host='rover-imu.local', port=PORT):
        self.host, self.port = host, port
        self.latest = None
        self.at = 0.0
        self.addr = None
        self.error = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _resolve(self):
        try:
            self.addr = socket.getaddrinfo(self.host, self.port, socket.AF_INET, socket.SOCK_DGRAM)[0][4]
            self.error = None
        except OSError as error:
            self.addr, self.error = None, f'cannot find {self.host}: {error}'

    def _loop(self):
        last_sub = 0.0
        while True:
            now = time.monotonic()
            if now - last_sub > SUB_EVERY_S:
                last_sub = now
                if not self.fresh():
                    self._resolve()           # the stick may have a new address after a reboot
                if self.addr:
                    try:
                        self.sock.sendto(b'sub', self.addr)
                    except OSError as error:
                        self.error = str(error)
            if not self.addr:
                time.sleep(0.5)
                continue
            try:
                data, _ = self.sock.recvfrom(512)
                self.latest, self.at = json.loads(data), time.monotonic()
            except (socket.timeout, ValueError):
                pass
            except OSError as error:
                self.error = str(error)
                time.sleep(0.2)

    def fresh(self):
        return self.latest is not None and time.monotonic() - self.at < STALE_S and bool(self.latest.get('cal'))

    def yaw(self):
        return self.latest['yaw']

    def rate(self):
        return self.latest['rate']

    def zero(self):
        if self.addr:
            self.sock.sendto(b'zero', self.addr)

    def status(self):
        d = self.latest or {}
        return {'online': self.fresh(), 'host': self.host, 'error': None if self.fresh() else self.error,
                'calibrated': bool(d.get('cal')), 'heading_deg': round(wrap(d['yaw']), 1) if d else None,
                'yaw_deg': d.get('yaw'), 'rate_dps': d.get('rate'), 'still': bool(d.get('still')),
                'bumps': d.get('bumps'), 'battery': d.get('bat'),
                'age_ms': round((time.monotonic() - self.at) * 1000) if d else None}
