"""Camera-only exploration: wander the apartment in small measured moves, building the floor map.

Run: python3 mac/explore.py [--steps 30]     (drive server, depth service and heading stick running)

Each step: pause so the map takes a view, read the smoothed left/centre/right free floor, then
  centre clear (>= FORWARD_CM)              -> forward STEP_CM, heading held by the gyro
  centre >= TURN_CM and a side >= SIDE_CM   -> 15° arc toward the more open side
  otherwise                                 -> multi-point turn toward the more open side: reverse
                                               arcs (heading swinging that way, over floor it just
                                               drove) and forward arcs where the camera says there's
                                               room, until it has turned TURN_AROUND_DEG or the way
                                               ahead is clear (a 2.65 m turning circle can't turn in
                                               a hallway any other way; it rocked back and forth before)
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
from pathlib import Path

DRIVE, DEPTH, DASH = 'http://127.0.0.1:8769', 'http://127.0.0.1:8774', 'http://127.0.0.1:8010'
STEP_CM = 25
FORWARD_CM, TURN_CM, SIDE_CM = 100, 60, 110
# tuned from your driving in shadow mode (mac/shadow.py, dashboard Shadow report -> Apply)
_tuned = Path(__file__).with_name('explore_params.json')
if _tuned.exists():
    globals().update({k: v for k, v in json.loads(_tuned.read_text()).items() if k in ('FORWARD_CM', 'TURN_CM', 'SIDE_CM')})
TURN_DEG = 15
GUARD_CM = 30
TURN_AROUND_DEG = 75
MIN_TURN_DEG = 20        # a multi-point turn turns at least this much before open floor can end it
FORWARD_MARGIN_CM = 10   # after a turn, drive on only with this much more than FORWARD_CM: readings near
                         # the threshold flicker (real run 2026-11-04 rocked back/forward 5 times)
REVERSE_DEG = 20
TURN_MOVES = 6
REVERSE_ARCS = False     # multi-point turns back up straight (over the path it came in on) and turn with
                         # forward arcs the camera watches: in the sim every collision (5 of 5) was the
                         # rear corner swinging into something unseen during a reverse arc
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
    return 'turn_' + ('left' if sec['left'] >= sec['right'] else 'right'), TURN_AROUND_DEG


def finished(out, kind=None, amount=None):
    """A move that completed. A turn that timed out having turned most of the way counts too
    (the sim showed runs ending on arcs a few degrees short)."""
    if out.get('error') or out.get('aborted'):
        return False
    if out.get('ended') == 'timeout' and kind in ('left', 'right', 'reverse_left', 'reverse_right'):
        return abs(out.get('turned_deg') or 0) >= STUCK_SHARE * amount
    return out.get('ended') in ('done', None)


def multi_point_turn(guard, side, target):
    """Reverse and forward arcs turning the heading toward `side`. Returns (moves, why it stopped)."""
    turned, moves = 0.0, []
    for k in range(TURN_MOVES):
        forward = k % 2 == 1
        if forward:
            time.sleep(SETTLE_S)
            d = call(DEPTH + '/status')
            if d.get('stop_for'):
                return moves, 'saw ' + ', '.join(d['stop_for'])
            sec = {n: v['cm'] for n, v in (d.get('sectors') or {}).items()}
            if not sec:
                return moves, 'depth not usable'
            if sec['centre'] >= FORWARD_CM + FORWARD_MARGIN_CM and turned >= MIN_TURN_DEG:
                return moves, 'facing open floor'
            if sec['centre'] < TURN_CM:
                continue                                # no room ahead: reverse again
        kind = side if forward else ('reverse_' + side if REVERSE_ARCS else 'back')
        amount = TURN_DEG if forward else (REVERSE_DEG if REVERSE_ARCS else STEP_CM)
        out = guard.move(kind, amount)
        moves.append((kind, out.get('turned_deg'), out.get('ended') or out.get('error')))
        if guard.why:
            return moves, f'guard: {guard.why}'
        if not finished(out, kind, amount):
            return moves, f'{kind} did not finish ({out.get("error") or out.get("ended")})'
        turned += abs(out.get('turned_deg') or 0) if kind != 'back' else 0
        if turned >= target:
            return moves, f'turned {turned:.0f}°'
    return moves, f'turned {turned:.0f}° in {TURN_MOVES} moves' 


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
        self.why, self.forward = None, kind in ('forward', 'left', 'right')
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
    global DRIVE, DEPTH
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--steps', type=int, default=30)
    parser.add_argument('--keep-map', action='store_true', help='add to the current map instead of a new one')
    parser.add_argument('--drive', default=DRIVE, help='drive server (the simulator uses http://127.0.0.1:9769)')
    parser.add_argument('--depth', default=DEPTH, help='depth service')
    parser.add_argument('--no-capture', action='store_true', help="don't record photos (simulator runs)")
    parser.add_argument('--reverse-arcs', action='store_true', help='multi-point turns reverse with the wheels turned')
    args = parser.parse_args()
    global REVERSE_ARCS
    DRIVE, DEPTH, REVERSE_ARCS = args.drive, args.depth, args.reverse_arcs or REVERSE_ARCS

    imu = call(DRIVE + '/imu')
    depth = call(DEPTH + '/status')
    if not imu.get('online'):
        raise SystemExit('heading stick offline or not calibrated (keep the rover still for a second)')
    if not depth.get('ok'):
        raise SystemExit(f'depth service not ready: {depth.get("why") or depth.get("error")}')
    if not args.keep_map:
        call(DRIVE + '/odometer/reset', {})
        call(DEPTH + '/map/reset', {})
    if not args.no_capture:
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
            age = d.get('frame_age_ms')
            if d.get('error') or not d.get('ok') or age is None or age > MAX_FRAME_AGE_MS:
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
            if kind.startswith('turn_'):
                moves, ended = multi_point_turn(guard, kind[5:], amount)
                print(json.dumps({'step': i, 'sectors': sec, 'move': f'{kind} {amount}', 'parts': moves,
                                  'ended': ended, 'odometer': call(DRIVE + '/status').get('odometer')}), flush=True)
                if ended.startswith(('saw', 'guard')):
                    call(DRIVE + '/arm', {'armed': True, 'fresh': True})
                    continue
                if 'did not finish' in ended or 'not usable' in ended:
                    why = ended
                    break
                continue
            out = guard.move(kind, amount)
            if guard.why:                               # the guard stopped it: re-arm and look again
                print(f'step {i}: guard stopped the {kind} ({guard.why})', flush=True)
                call(DRIVE + '/arm', {'armed': True, 'fresh': True})
                continue
            print(json.dumps({'step': i, 'sectors': sec, 'move': f'{kind} {amount}', 'ended': out.get('ended'),
                              'error': out.get('error'), 'odometer': out.get('odometer'),
                              'map': d.get('map')}), flush=True)
            if 'acknowledge stop' in str(out.get('error')):  # a missed radio reply: stop again, go on if it acks
                acked = False
                for _ in range(3):
                    again = call(DRIVE + '/stop', {'from': 'explorer retry'})
                    if not again.get('error') and again.get('armed') is False:
                        acked = True
                        break
                    time.sleep(0.3)
                if acked:
                    print(f'step {i}: stop ACK missed, a retry was acknowledged', flush=True)
                    call(DRIVE + '/arm', {'armed': True, 'fresh': True})
                    continue
            if kind in ('left', 'right') and out.get('ended') == 'timeout' and \
                    abs(out.get('turned_deg') or 0) < STUCK_SHARE * amount:
                why = f'steering looks stuck: turned {out.get("turned_deg")}° of {amount}°'
                break
            if not finished(out, kind, amount):
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
        if not args.no_capture:
            call(DASH + '/capture', {'on': False})
    print('stopped:', why)
    print('map:', call(DEPTH + '/map'))
    if scale:
        print(f'scale check: true/estimated distance = {statistics.median(scale):.2f} from {len(scale)} moves '
              '(multiply depth_service.K_PX_CM by this)')


if __name__ == '__main__':
    main()
