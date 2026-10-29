"""Drive server: the single owner of the transmitter's USB port, with an arm lease.

Run: python3 drive/drive_server.py        (HTTP on 127.0.0.1:8769)

  GET  /status                     armed, busy, last pulse, receiver app
  POST /arm    {"armed": true}     arm, or renew the lease (expires after LEASE_S without renewal)
  POST /arm    {"armed": false}    disarm and stop
  POST /pulse  {"key": "W", "ms": 300}   bounded pulse; refused (403) unless armed, (409) during manual driving
  POST /hold   {"key": "W"}        manual hold-to-drive: drives while renewed every ~150 ms, stops with
                                   ACK HOLD_S after the last renewal; refused (403) unless armed
  POST /stop                       abort any pulse or hold, disarm, stop with ACK
  GET  /calibration                saved calibration, the app version it makes, the installed app
  POST /calibration {values, install}   save (validated); install=true disarms, then installs the
                                   calibrated app on the receiver with the signed update

Motion only happens while armed, and the arm lease must be renewed every second or so
(the dashboard page does this while its Arm switch is on). If the page closes, the network
drops, or the client hangs, the lease expires, the server disarms and stops. The receiver's
own 500 ms watchdog stops the motors if this process dies.
"""
import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibration  # noqa: E402

LEASE_S = 3.0
PULSE_LIMITS_MS = {'W': 600, 'S': 400, 'A': 600, 'D': 600}
HOLD_S = 0.35                 # manual drive stops this long after the last /hold renewal
MANUAL_QUIET_S = 2.0          # autonomous pulses refused this long after manual driving
REPEAT_S = 0.15               # resend the key this often (receiver watchdog is 500 ms)


class DriveState:
    def __init__(self, tx):
        self.tx = tx
        self.lock = threading.Lock()          # serial access
        self.abort = threading.Event()
        self.armed_until = 0.0
        self.busy = False
        self.last = None
        self.error = None
        self.hold_key = None
        self.hold_until = 0.0
        self.manual_until = 0.0
        self.holding = False
        self.installed = None         # receiver app|running, refreshed after installs

    @property
    def armed(self):
        return time.monotonic() < self.armed_until

    def arm(self, on):
        if on:
            self.armed_until = time.monotonic() + LEASE_S
            self.abort.clear()
        else:
            self.stop('disarmed')

    def stop(self, why='stop'):
        self.armed_until = 0.0
        self.abort.set()
        with self.lock:
            try:
                self.tx.stop()
                self.error = None
            except Exception as error:
                self.error = f'stop failed: {error}'
        self.last = {'key': 'X', 'why': why, 't': time.time()}

    def pulse(self, key, ms):
        if not self.armed:
            raise PermissionError('not armed')
        if time.monotonic() < self.manual_until:
            raise RuntimeError('manual driving')
        if key not in PULSE_LIMITS_MS:
            raise ValueError('key must be W, A, S or D')
        ms = max(0, min(int(ms), PULSE_LIMITS_MS[key]))
        with self.lock:
            if not self.armed or self.abort.is_set():
                raise PermissionError('not armed')
            self.busy = True
            started = time.monotonic()
            try:
                self.tx.pulse(key, ms, abort=self.abort)
            finally:
                self.busy = False
        took = round((time.monotonic() - started) * 1000)
        self.last = {'key': key, 'ms': ms, 'stopped_after_ms': took, 't': time.time()}
        return self.last

    def hold(self, key):
        """Manual hold-to-drive: start or renew. Drives until HOLD_S passes without renewal,
        the lease lapses, or stop() is called; always ends with an ACKed stop."""
        if not self.armed:
            raise PermissionError('not armed')
        if key not in PULSE_LIMITS_MS:
            raise ValueError('key must be W, A, S or D')
        now = time.monotonic()
        self.hold_key = key
        self.hold_until = now + HOLD_S
        self.manual_until = now + HOLD_S + MANUAL_QUIET_S
        if not self.holding:
            self.holding = True
            threading.Thread(target=self._hold_loop, daemon=True).start()
        return {'holding': key}

    def _hold_loop(self):
        try:
            with self.lock:
                self.busy = True
                try:
                    while (self.armed and not self.abort.is_set()
                           and time.monotonic() < self.hold_until):
                        self.tx.wire(self.hold_key)
                        self.last = {'key': self.hold_key, 'manual': True, 't': time.time()}
                        self.abort.wait(REPEAT_S)
                finally:
                    try:
                        self.tx.stop()
                    except Exception as error:
                        self.error = f'stop failed: {error}'
                    self.busy = False
        finally:
            self.holding = False

    def calibration(self):
        saved = calibration.load()
        return {'saved': saved, 'expected_version': calibration.version(saved), 'installed': self.installed,
                'limits': calibration.LIMITS}

    def save_calibration(self, values, install):
        saved = calibration.save(values)
        if install:
            self.stop('installing calibration')
            source = calibration.render(saved)
            with self.lock:
                self.busy = True
                try:
                    self.tx.update_app(source)
                    self.installed = self.tx.status()
                finally:
                    self.busy = False
        return self.calibration()

    def watchdog(self):
        was_armed = False
        while True:
            armed = self.armed
            if was_armed and not armed:
                self.stop('arm lease expired')
            was_armed = armed
            time.sleep(0.2)

    def status(self):
        return {'armed': self.armed, 'busy': self.busy, 'holding': self.hold_key if self.holding else None,
                'installed': self.installed,
                'last': self.last, 'error': self.error, 'lease_s': LEASE_S, 'pulse_limits_ms': PULSE_LIMITS_MS}


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def body(self):
            size = int(self.headers.get('Content-Length', 0))
            return json.loads(self.rfile.read(size) or b'{}') if size else {}

        def do_GET(self):
            if self.path == '/status':
                return self.reply(200, state.status())
            if self.path == '/calibration':
                return self.reply(200, state.calibration())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                if self.path == '/arm':
                    state.arm(bool(self.body().get('armed')))
                    return self.reply(200, state.status())
                if self.path == '/stop':
                    state.stop('stop requested')
                    return self.reply(200, state.status())
                if self.path == '/pulse':
                    b = self.body()
                    return self.reply(200, state.pulse(str(b.get('key', '')).upper(), b.get('ms', 0)))
                if self.path == '/calibration':
                    b = self.body()
                    return self.reply(200, state.save_calibration(b.get('values') or {}, bool(b.get('install'))))
                if self.path == '/hold':
                    return self.reply(200, state.hold(str(self.body().get('key', '')).upper()))
                self.reply(404, {'error': 'not found'})
            except PermissionError as error:
                self.reply(403, {'error': str(error)})
            except RuntimeError as error:
                self.reply(409, {'error': str(error)})
            except ValueError as error:
                self.reply(400, {'error': str(error)})
            except Exception as error:
                state.error = f'{type(error).__name__}: {error}'
                self.reply(500, {'error': state.error})

        def log_message(self, *args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8769)
    args = parser.parse_args()
    from rover_drive import Transmitter
    tx = Transmitter()
    state = DriveState(tx)
    state.installed = tx.status()
    threading.Thread(target=state.watchdog, daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(state))
    print(f'drive server on http://127.0.0.1:{args.port} (disarmed; receiver app {state.installed})', flush=True)
    try:
        server.serve_forever()
    finally:
        state.stop('server exit')


if __name__ == '__main__':
    main()
