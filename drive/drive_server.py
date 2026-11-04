"""Drive server: the single owner of the transmitter's USB port, with an arm lease.

Run: python3 drive/drive_server.py        (HTTP on 127.0.0.1:8769)

  GET  /status                     armed, busy, last pulse, receiver app
  POST /arm    {"armed": true, "fresh": true}   arm (expires after LEASE_S without renewal); an arm
                                   without "fresh" or "renew" is refused (an old page re-arming after a stop)
  POST /arm    {"armed": true, "renew": true}   renew only: refused once disarmed or stopped, so a
                                   page or script that is still renewing can't undo a stop
  POST /arm    {"armed": false}    disarm and stop
  POST /pulse  {"key": "W", "ms": 300}   bounded pulse; refused (403) unless armed, (409) during manual driving
  POST /hold   {"key": "W"}        manual hold-to-drive: drives while renewed every ~150 ms, stops with
                                   ACK HOLD_S after the last renewal; refused (403) unless armed
  POST /stop                       abort any pulse or hold, disarm, stop with ACK
  POST /move   {"kind": "forward"|"back", "amount": cm}  or  {"kind": "left"|"right", "amount": deg}
                                   measured move (timed from drive/motion.json; refused unless armed);
                                   "heading": deg (forward/back): hold this gyro heading instead of the
                                   heading the move starts at; such a straight run may last up to
                                   motion.LONG_MOVE_S (30 s) instead of 8
                                   {"kind": "reverse_left"|"reverse_right", "amount": deg} backs up turning the
                                   heading that way (for multi-point turns);
                                   {"kind": "test_forward"|"test_reverse"|"test_arc"} = the timed run for measuring
                                   With the heading stick online (and hold on): forward/back steer to keep the
                                   starting heading; left/right turn until the stick has turned that many degrees.
                                   Manual W/S holds do the same; arming zeroes the heading
  GET  /imu                        heading stick: online, heading, turn rate, bumps, battery; hold on/off
  POST /imu    {"hold": bool}      heading hold on/off;   POST /imu/zero   heading = 0 here
  GET  /motion                     measured speeds/turning circle, odometer (estimates)
  POST /motion {values}            save measurements;  POST /odometer/reset
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
import imu as heading  # noqa: E402
import motion  # noqa: E402

LEASE_S = 3.0
PULSE_LIMITS_MS = {'W': 600, 'S': 400, 'A': 600, 'D': 600, 'L': 2000, 'R': 2000, 'Z': 400, 'C': 400}
# A/D: forward arcs; L/R: steer only (wheels don't drive); Z/C: reverse while steering left/right
HOLD_S = 0.35                 # manual drive stops this long after the last /hold renewal
MANUAL_QUIET_S = 2.0          # autonomous pulses refused this long after manual driving
REPEAT_S = 0.15               # resend the key this often (receiver watchdog is 500 ms)
TICK_S = 0.1                  # heading-hold control step
TURN_TIMEOUT = 1.6            # gyro turns give up after this many times the planned time
BLOCKED_DEG = 15.0            # a straight move this far off its heading has hit something: stop at once
                              # (2026-11-04: it hit a chair base, swung to -55° and kept pushing)


class DriveState:
    def __init__(self, tx, imu=None):
        self.tx = tx
        self.imu = imu
        self.heading_hold = True
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
        self.motion = motion.load()
        self.odometer = motion.Odometer()

    @property
    def armed(self):
        return time.monotonic() < self.armed_until

    def arm(self, on, renew=False):
        if on and renew and not self.armed:
            return False
        if on:
            if not self.armed and self.imu:          # arming (not a lease renewal): heading = 0 here
                self.imu.zero()
            self.armed_until = time.monotonic() + LEASE_S
            self.abort.clear()
        else:
            self.stop('disarmed')
        return True

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
        if why not in ('test reset',):
            print(f'{time.strftime("%H:%M:%S")} stop: {why}', flush=True)   # in drive-server.log

    def power(self):
        live = calibration.live_values(self.installed)
        return live['speed'] if live else calibration.load()['speed']

    def count(self, key, seconds, turn_deg=None):
        self.odometer.add(key, seconds, self.motion, self.power(), turn_deg)

    def bumps(self):
        """The heading stick's bump count (jolts over 0.6 g), or 0 without one."""
        try:
            return int((self.imu.status() or {}).get('bumps') or 0) if self.imu else 0
        except Exception:
            return 0

    def gyro(self):
        """The heading stick, when it's online and heading hold is on."""
        return self.imu if self.imu and self.heading_hold and self.imu.fresh() else None

    def imu_status(self):
        base = self.imu.status() if self.imu else {'online': False, 'error': 'no heading stick configured'}
        return dict(base, hold=self.heading_hold)

    def _gyro_move(self, kind, amount, key, seconds, g, target=None):
        """Closed loop on the stick's heading. Returns (driven s, how it ended)."""
        throttle, _ = motion.KEYS[key]
        turning = kind in ('left', 'right', 'reverse_left', 'reverse_right')
        sign = 1 if kind in ('left', 'reverse_left') else -1
        limit = seconds * TURN_TIMEOUT if turning else seconds
        start = prev_yaw = g.yaw()
        hold_to = start if target is None else float(target)   # straight moves: this heading (absolute)
        started = prev_t = time.monotonic()
        sent, ended = key, 'done'
        moved_at, moved_yaw = started, start         # stall check for turns
        bumps0 = self.bumps()
        self.gyro_log = []                            # (s, key sent, yaw) per step
        hold = heading.HeadingHold(throttle, motion.GENTLE)
        while True:
            if self.abort.is_set():
                ended = 'aborted'
                break
            if not g.fresh():
                ended = 'heading stick lost'
                break
            now, yaw, rate = time.monotonic(), g.yaw(), g.rate()
            dt = now - prev_t
            self.count(sent, dt, yaw - prev_yaw)
            prev_t, prev_yaw = now, yaw
            if self.bumps() > bumps0:
                ended = 'blocked: bump'
                break
            if not turning and abs(hold_to - yaw) > BLOCKED_DEG:
                ended = f'blocked: knocked {yaw - hold_to:+.0f}° off course'
                break
            if turning:
                if sign * (yaw - start) + max(0.0, sign * rate) * heading.TURN_LEAD_S >= amount:
                    break
                if abs(yaw - moved_yaw) > heading.STALL_DPS * heading.STALL_S / 2:
                    moved_at, moved_yaw = now, yaw
                elif now - moved_at > heading.STALL_S + 0.5:     # +0.5 s to get going
                    ended = 'stuck: heading not changing'
                    break
                sent = key
            else:
                sent = hold.key(hold_to - yaw, rate, dt)
            if now - started >= limit:
                ended = 'timeout' if turning else 'done'
                break
            self.tx.wire(sent)
            self.gyro_log.append((round(now - started, 2), sent, round(yaw - start, 1)))
            self.abort.wait(TICK_S)
        self.gyro_end = (sent, prev_t, prev_yaw, g)
        return time.monotonic() - started, ended

    def move(self, kind, amount=None, heading=None):
        """A measured move: drive `key` for the planned time, then stop with ACK."""
        if not self.armed:
            raise PermissionError('not armed')
        if time.monotonic() < self.manual_until or self.holding:
            raise RuntimeError('manual driving')
        tests = {'test_forward': ('W', 'forward'), 'test_reverse': ('S', 'reverse'), 'test_arc': ('A', 'arc')}
        if kind in tests:
            key, seconds = tests[kind][0], motion.TEST_S[tests[kind][1]]
        else:
            if not isinstance(amount, (int, float)) or isinstance(amount, bool) or amount <= 0:
                raise ValueError('amount must be a positive number (cm or degrees)')
            key, seconds = motion.plan(kind, amount, self.motion, self.power(),
                                       long=heading is not None and self.gyro() is not None)
        with self.lock:
            if not self.armed or self.abort.is_set():
                raise PermissionError('not armed')
            self.busy = True
            started = time.monotonic()
            g = self.gyro() if kind in ('forward', 'back', 'left', 'right', 'reverse_left', 'reverse_right') else None
            start_yaw = g.yaw() if g else None
            ended = 'done'
            self.gyro_end = None
            try:
                if g:
                    driven, ended = self._gyro_move(kind, amount, key, seconds, g,
                                                    heading if kind in ('forward', 'back') else None)
                else:
                    while not self.abort.is_set():
                        self.tx.wire(key)
                        remaining = seconds - (time.monotonic() - started)
                        if remaining <= 0:
                            break
                        self.abort.wait(min(REPEAT_S, remaining))
                    driven = min(time.monotonic() - started, seconds)
            finally:
                try:
                    self.tx.stop()
                finally:
                    self.busy = False
                    if g:                               # the last step, with the heading the stick saw
                        sent, t, yaw, _ = self.gyro_end or (key, started, start_yaw, g)
                        end_yaw = g.yaw() if g.fresh() else yaw
                        self.count(sent, max(0.0, time.monotonic() - t), end_yaw - yaw)
                    else:
                        self.count(key, min(time.monotonic() - started, seconds))
        self.last = {'key': key, 'move': kind, 'amount': amount, 'ms': round(driven * 1000), 't': time.time(),
                     'aborted': self.abort.is_set(), 'gyro': bool(g), 'ended': ended}
        if g and g.fresh():
            moved = g.yaw() - start_yaw
            self.last['turned_deg'] = round(moved, 1)
        if g:
            self.last['steps'] = self.gyro_log
        return dict(self.last, odometer=self.odometer.as_dict())

    def motion_status(self):
        return {'values': self.motion, 'power': self.power(), 'test_s': motion.TEST_S, 'max_move_s': motion.MAX_MOVE_S,
                'odometer': self.odometer.as_dict()}

    def save_motion(self, values):
        self.motion = motion.save(values)
        return self.motion_status()

    def pulse(self, key, ms):
        if not self.armed:
            raise PermissionError('not armed')
        if time.monotonic() < self.manual_until:
            raise RuntimeError('manual driving')
        if key not in PULSE_LIMITS_MS:
            raise ValueError('key must be W, A, S, D, L, R, Z or C')
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
                self.count(key, min(time.monotonic() - started, ms / 1000))
        took = round((time.monotonic() - started) * 1000)
        self.last = {'key': key, 'ms': ms, 'stopped_after_ms': took, 't': time.time()}
        return self.last

    def hold(self, key):
        """Manual hold-to-drive: start or renew. Drives until HOLD_S passes without renewal,
        the lease lapses, or stop() is called; always ends with an ACKed stop."""
        if not self.armed:
            raise PermissionError('not armed')
        if key not in PULSE_LIMITS_MS:
            raise ValueError('key must be W, A, S, D, L, R, Z or C')
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
                    # Straight W/S with the heading stick online (and hold on): keep the heading the
                    # straight run started with; steering keys pass through, and the next straight run
                    # locks the new heading (where the turn is heading to settle).
                    sent, prev_yaw, target, hold = None, None, None, None
                    while (self.armed and not self.abort.is_set()
                           and time.monotonic() < self.hold_until):
                        key = self.hold_key
                        g = self.gyro()
                        now = time.monotonic()
                        yaw = g.yaw() if g else None
                        if g and key in ('W', 'S'):
                            throttle = 1 if key == 'W' else -1
                            if hold is None or hold.throttle != throttle:
                                target, hold = yaw + g.rate() * heading.LEAD_S, heading.HeadingHold(throttle, motion.GENTLE)
                            wire = hold.key(target - yaw, g.rate(), now - sent[1] if sent else TICK_S)
                        else:
                            target = hold = None
                            wire = key
                        self.tx.wire(wire)
                        if sent:                    # odometer: time since the previous frame
                            turned = yaw - prev_yaw if yaw is not None and prev_yaw is not None else None
                            self.count(sent[0], min(now - sent[1], 2 * REPEAT_S), turned)
                        sent, prev_yaw = (wire, now), yaw
                        self.last = {'key': key, 'manual': True, 'heading_hold': hold is not None, 't': time.time()}
                        self.abort.wait(TICK_S if hold else REPEAT_S)
                    if sent:
                        self.count(sent[0], min(time.monotonic() - sent[1], 2 * REPEAT_S))
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
                'live': calibration.live_values(self.installed),
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
                    self.tx.stop()      # the new app only moves the servo on a command: centre it now
                finally:
                    self.busy = False
        return self.calibration()

    def live_calibration(self, values=None, action=None):
        """Change centre/range/speed on the rover at once (no install), or save/undo them there."""
        if self.holding:
            raise RuntimeError('manual driving')
        with self.lock:
            live = calibration.live_values(self.installed)
            if live is None or not (self.installed or '').startswith(calibration.BASE_VERSION + '+'):
                # Codes for another app version can read as drive keys (0.6's codes reversed 0.7).
                raise RuntimeError(f'the rover runs {(self.installed or "?").split("|")[0]}, not '
                                   f'{calibration.BASE_VERSION}: install it first (Wiring -> Install)')
            if action in ('save', 'revert'):
                self.tx.codes([calibration.SAVE_CODE if action == 'save' else calibration.REVERT_CODE])
            elif values:
                target = calibration.validate(dict(calibration.load(), **{k: v for k, v in values.items()
                                                                           if k in calibration.LIVE_CODES}))
                self.tx.codes(calibration.codes_to(live, target))
            self.installed = self.tx.status()
        if action == 'save':
            calibration.sync_from_rover(self.installed)
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
                'last': self.last, 'error': self.error, 'lease_s': LEASE_S, 'pulse_limits_ms': PULSE_LIMITS_MS,
                'odometer': self.odometer.as_dict(), 'imu': self.imu_status()}


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
            if self.path == '/motion':
                return self.reply(200, state.motion_status())
            if self.path == '/imu':
                return self.reply(200, state.imu_status())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                if self.path == '/arm':
                    b = self.body()
                    if b.get('armed') and not (b.get('fresh') or b.get('renew')):
                        return self.reply(409, {'error': 'reload the page (old arm request)'})
                    state.arm(bool(b.get('armed')), renew=bool(b.get('renew')))
                    return self.reply(200, state.status())
                if self.path == '/stop':
                    state.stop(f'stop requested ({self.body().get("from") or "unnamed"})')
                    return self.reply(200, state.status())
                if self.path == '/pulse':
                    b = self.body()
                    return self.reply(200, state.pulse(str(b.get('key', '')).upper(), b.get('ms', 0)))
                if self.path == '/move':
                    b = self.body()
                    return self.reply(200, state.move(str(b.get('kind', '')), b.get('amount'), b.get('heading')))
                if self.path == '/motion':
                    return self.reply(200, state.save_motion(self.body().get('values') or {}))
                if self.path == '/imu':
                    state.heading_hold = bool(self.body().get('hold', True))
                    return self.reply(200, state.imu_status())
                if self.path == '/imu/zero':
                    if state.imu:
                        state.imu.zero()
                    return self.reply(200, state.imu_status())
                if self.path == '/odometer/reset':
                    state.odometer.reset()
                    return self.reply(200, state.motion_status())
                if self.path == '/calibration/live':
                    b = self.body()
                    return self.reply(200, state.live_calibration(b.get('values'), b.get('action')))
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
    parser.add_argument('--imu-host', default='rover-imu.local', help="heading stick ('' = none)")
    args = parser.parse_args()
    from rover_drive import Transmitter
    tx = Transmitter()
    state = DriveState(tx, heading.Imu(args.imu_host).start() if args.imu_host else None)
    state.installed = tx.status()
    if calibration.sync_from_rover(state.installed):   # remote-calibrated centre/range/speed
        print('calibration.json updated from the rover', flush=True)
    threading.Thread(target=state.watchdog, daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(state))
    print(f'drive server on http://127.0.0.1:{args.port} (disarmed; receiver app {state.installed})', flush=True)
    try:
        server.serve_forever()
    finally:
        state.stop('server exit')


if __name__ == '__main__':
    main()
