"""Camera-only exploration: wander the apartment in small measured moves, building the floor map.

Run: python3 mac/explore.py [--steps 30]     (drive server, depth service and heading stick running)

Each step: pause so the map takes a view, read the smoothed left/centre/right free floor, then
  centre clear (>= FORWARD_CM)              -> forward STEP_CM, heading held by the gyro
  centre >= TURN_CM and a side >= SIDE_CM   -> 15° arc toward the more open side
  otherwise                                 -> back STEP_CM
Never drives while a dog, cat or person is in view (depth_service "stop_for"): it waits up to
WAIT_S for them to leave, then gives up. During every move a guard watches ~10 times a second and
stops the rover at once if one appears or the centre gets nearer than GUARD_CM.
Stops for good (and disarms) on a bump, the heading stick or depth going stale, a move that didn't
finish, a turn that barely turned (steering stuck), or the step limit. Recording photos is
switched on for the run.

Scale check: when the centre reading has a finite distance before and after a straight forward
move, the drop in the estimate is compared with the distance driven; the ratio corrects the depth
service's --bottom-cm guess (printed at the end).
"""
import argparse
import json
import statistics
import threading
import time
import urllib.error
import urllib.request

DRIVE, DEPTH, DASH = 'http://127.0.0.1:8769', 'http://127.0.0.1:8774', 'http://127.0.0.1:8010'
STEP_CM = 25
FORWARD_CM, TURN_CM, SIDE_CM = 100, 60, 110
TURN_DEG = 15
GUARD_CM = 30
WAIT_S = 30
STUCK_SHARE = 0.4        # a turn that reached less than this share of its angle: steering stuck
SETTLE_S = 1.4           # the depth service maps after 0.6 s still; smoothing needs ~5 fresh frames
MAX_FRAME_AGE_MS = 400


def call(url, body=None, timeout=20):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data, {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'http': e.code, 'error': e.read().decode(errors='replace')}
    except OSError as e:
        return {'error': str(e)}


def choose(sec):
    if sec['centre'] >= FORWARD_CM:
        return 'forward', STEP_CM
    if sec['centre'] >= TURN_CM and max(sec['left'], sec['right']) >= SIDE_CM:
        return ('left' if sec['left'] >= sec['right'] else 'right'), TURN_DEG
    return 'back', STEP_CM


class Guard:
    """Watches the depth service during a move; stops the rover the moment it must."""
    def __init__(self):
        self.why = None
        self.active = threading.Event()
        self.forward = True
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            self.active.wait()
            d = call(DEPTH + '/status', timeout=0.5)
            why = None
            if d.get('stop_for'):
                why = 'saw ' + ', '.join(d['stop_for'])
            elif self.forward and d.get('ok') and d['sectors']['centre']['cm'] < GUARD_CM:
                why = f'centre {d["sectors"]["centre"]["cm"]} cm'
            if why and self.active.is_set():
                self.why = why                          # before the stop: the move returns as soon as it lands
                self.active.clear()
                call(DRIVE + '/stop', {'from': f'explorer guard: {why}'})
            time.sleep(0.1)

    def move(self, kind, amount):
        self.why, self.forward = None, kind != 'back'
        self.active.set()
        try:
            return call(DRIVE + '/move', {'kind': kind, 'amount': amount})
        finally:
            self.active.clear()


def wait_for_clear_view():
    """None once no dog/cat/person is in view, else what is still there after WAIT_S."""
    end = time.monotonic() + WAIT_S
    while True:
        seen = call(DEPTH + '/status').get('stop_for') or []
        if not seen:
            return None
        if time.monotonic() > end:
            return ', '.join(seen)
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--steps', type=int, default=30)
    parser.add_argument('--keep-map', action='store_true', help='add to the current map instead of a new one')
    args = parser.parse_args()

    imu = call(DRIVE + '/imu')
    depth = call(DEPTH + '/status')
    if not imu.get('online'):
        raise SystemExit('heading stick offline or not calibrated (keep the rover still for a second)')
    if not depth.get('ok'):
        raise SystemExit(f'depth service not ready: {depth.get("why") or depth.get("error")}')
    if not args.keep_map:
        call(DRIVE + '/odometer/reset', {})
        call(DEPTH + '/map/reset', {})
    call(DASH + '/capture', {'on': True})
    bumps0 = imu['bumps']
    done = threading.Event()

    def lease():
        while not done.is_set():
            call(DRIVE + '/arm', {'armed': True, 'renew': True}, timeout=2)
            done.wait(0.5)
    call(DRIVE + '/arm', {'armed': True, 'fresh': True})              # arming zeroes the heading
    threading.Thread(target=lease, daemon=True).start()
    scale, why = [], 'step limit'
    guard = Guard()
    try:
        for i in range(args.steps):
            time.sleep(SETTLE_S)
            d, g = call(DEPTH + '/status'), call(DRIVE + '/imu')
            if d.get('error') or not d.get('ok') or (d.get('frame_age_ms') or 10 ** 6) > MAX_FRAME_AGE_MS:
                why = f'depth not usable ({d.get("why") or d.get("error") or d.get("frame_age_ms")})'
                break
            if not g.get('online'):
                why = 'heading stick offline'
                break
            if (g.get('bumps') or 0) > bumps0:
                why = 'bump'
                break
            if d.get('stop_for'):
                print(f'step {i}: waiting, I see {", ".join(d["stop_for"])}', flush=True)
                still = wait_for_clear_view()
                if still:
                    why = f'{still} stayed in view for {WAIT_S} s'
                    break
                continue
            sec = {k: v['cm'] for k, v in d['sectors'].items()}
            kind, amount = choose(sec)
            out = guard.move(kind, amount)
            if guard.why:                               # the guard stopped it: re-arm and look again
                print(f'step {i}: guard stopped the {kind} ({guard.why})', flush=True)
                call(DRIVE + '/arm', {'armed': True, 'fresh': True})
                continue
            print(json.dumps({'step': i, 'sectors': sec, 'move': f'{kind} {amount}', 'ended': out.get('ended'),
                              'error': out.get('error'), 'odometer': out.get('odometer'),
                              'map': d.get('map')}), flush=True)
            if 'acknowledge stop' in str(out.get('error')):  # a missed radio reply: stop again, go on if it acks
                again = call(DRIVE + '/stop', {'from': 'explorer'})
                if not again.get('error') and again.get('armed') is False:
                    print(f'step {i}: stop ACK missed once, the retry was acknowledged', flush=True)
                    call(DRIVE + '/arm', {'armed': True, 'fresh': True})
                    continue
            if kind in ('left', 'right') and out.get('ended') == 'timeout' and \
                    abs(out.get('turned_deg') or 0) < STUCK_SHARE * amount:
                why = f'steering looks stuck: turned {out.get("turned_deg")}° of {amount}°'
                break
            if out.get('error') or out.get('aborted') or out.get('ended') not in ('done', None):
                why = f'move did not finish ({out.get("error") or out.get("ended") or "aborted"})'
                break
            if kind == 'forward' and sec['centre'] < 250:
                time.sleep(SETTLE_S)
                after = call(DEPTH + '/status').get('sectors', {}).get('centre', {}).get('cm')
                if after and after < 250 and sec['centre'] - after > 3:
                    scale.append(amount / (sec['centre'] - after))
    finally:
        done.set()
        time.sleep(0.6)                                 # let a renewal in flight land before disarming
        call(DRIVE + '/arm', {'armed': False})
        call(DASH + '/capture', {'on': False})
    print('stopped:', why)
    print('map:', call(DEPTH + '/map'))
    if scale:
        print(f'scale check: true/estimated distance = {statistics.median(scale):.2f} from {len(scale)} moves '
              '(multiply depth_service.K_PX_CM by this)')


if __name__ == '__main__':
    main()
