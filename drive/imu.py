"""Heading from the M5StickS3 on the rover (firmware/m5stick-imu), over UDP port 8773.

The stick streams {"yaw", "rate", "still", "cal", "bumps", "bat", ...} ~50 times a second to
whoever sent it "sub" in the last 3 s; yaw is unwrapped degrees, + = turned left.

Heading hold: while driving straight, steer toward the heading the move started with.
The steering keys are all-or-nothing (full lock), so the error is predicted LEAD_S ahead from
the turn rate to stop before overshooting, and small errors inside DEADBAND_DEG are left alone.
"""
import json
import socket
import threading
import time

PORT = 8773
STALE_S = 0.3            # no packet for this long = offline
SUB_EVERY_S = 1.0
DEADBAND_DEG = 2.0
LEAD_S = 0.25            # servo + radio + Wi-Fi delay, roughly


def steer_key(throttle, error_deg, rate_dps=0.0, deadband=DEADBAND_DEG, lead_s=LEAD_S):
    """Key that steers toward the target. error = target - yaw (+ = need to turn left).
    Forward: A turns left, D right. Reversing turns the other way: C swings the heading left."""
    predicted = error_deg - rate_dps * lead_s
    straight = 'W' if throttle > 0 else 'S'
    if abs(predicted) <= deadband:
        return straight
    left = predicted > 0
    if throttle > 0:
        return 'A' if left else 'D'
    return 'C' if left else 'Z'


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
