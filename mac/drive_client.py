"""Turns dog-watcher decisions into bounded drive pulses via drive/drive_server.py.

Move -> stop -> observe: after every pulse the caller must see OBSERVE_S of fresh frames
captured after the stop before the next move. The drive server refuses pulses unless the
dashboard has armed it, so with the switch off this only records what it would do.
"""
import json
import time
import urllib.request

DRIVE_URL = 'http://127.0.0.1:8769'
OBSERVE_S = 0.6
# intent -> (key, ms). Short, conservative pulses; tune on the assembled chassis.
# Car steering, keys W/A/S/D only: turns are forward arcs.
PULSES = {
    'forward': ('W', 300),
    'back': ('S', 250),
    'turn_left': ('A', 300),
    'turn_right': ('D', 300),
}
# Scanning rotates roughly in place: forward arc, then straight reverse, repeated. Each pair
# turns the rover a few degrees with little net travel (no reverse-with-steering on the receiver).
SCANS = {
    'scan_left': [('A', 500), ('S', 300)],     # ~19 deg per pair in the simulator
    'scan_right': [('D', 500), ('S', 300)],
}
STOPS = {'stop', 'park'}


def _post(path, body=None, timeout=3.0):
    req = urllib.request.Request(DRIVE_URL + path, json.dumps(body or {}).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.load(resp)


def _get(path, timeout=1.0):
    with urllib.request.urlopen(DRIVE_URL + path, timeout=timeout) as resp:
        return json.load(resp)


class DriveExecutor:
    def __init__(self, post=_post, get=_get):
        self.post, self.get = post, get
        self.observe_until = 0.0
        self.captured_after = None   # Mac monotonic time of the last stop
        self.last = None             # last executed (or skipped) action, for status/logging
        self.stopped = True
        self.scan_step = 0           # position in the scan's arc/reverse cycle
        self.last_key = None         # key of the last pulse sent

    def ready(self, now):
        return now >= self.observe_until

    def armed(self):
        try:
            return bool(self.get('/status').get('armed'))
        except OSError:
            return False

    def act(self, action, now):
        """Execute a decision. Returns a dict describing what happened."""
        if action in STOPS:
            if not self.stopped:
                try:
                    self.post('/stop')
                except OSError:
                    pass
                self.stopped = True
            self.last = {'action': action, 'sent': 'X'}
            return self.last
        if action not in PULSES and action not in SCANS:
            self.last = {'action': action, 'sent': None}
            return self.last
        if not self.ready(now):
            return {'action': action, 'sent': None, 'why': 'observing'}
        if action in SCANS:
            cycle = SCANS[action]
            key, ms = cycle[self.scan_step % len(cycle)]
        else:
            key, ms = PULSES[action]
        if not self.armed():
            self.last = {'action': action, 'sent': None, 'why': 'disarmed', 'would_send': f'{key} {ms} ms'}
            return self.last
        try:
            result = self.post('/pulse', {'key': key, 'ms': ms}, timeout=5.0)
            self.stopped = True       # every pulse ends with an ACKed stop
            done = time.monotonic()
            self.captured_after = done
            self.observe_until = done + OBSERVE_S
            self.scan_step = self.scan_step + 1 if action in SCANS else 0
            self.last_key = key
            self.last = {'action': action, 'sent': f'{key} {ms} ms', 'result': result}
        except OSError as error:
            self.last = {'action': action, 'sent': None, 'why': f'drive server: {error}'}
        return self.last
