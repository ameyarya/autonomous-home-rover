"""Drive a planned tour of the simulated apartment: "go to the kitchen", then the next room...

Run: sim/.venv/bin/python sim/goto_tour.py [--rooms kitchen,bathroom,bedroom,living,home]
The rover starts at HOME with a known pose (on the real rover: a known spot, e.g. by the dock).
Each leg: plan a route with mac/route_planner.py (Hybrid A* with the measured 132.5 cm turning
radius, body clear of walls, furniture and virtual doors), drive the first piece of it through the
real drive server (heading held by the gyro, turns stopped at the angle), work out where it thinks
it is from its own odometer (gyro heading + drive-time distance; never the simulator's truth),
re-plan, repeat until it believes it has arrived. The truth is only used to score each leg.
"""
import argparse
import json
import math
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'mac'))
import route_planner as P  # noqa: E402
from localizer import Localizer  # noqa: E402

ROOMS = {'home': (2.8, 4.6), 'kitchen': (1.8, 1.3), 'bathroom': (5.2, 2.3), 'bedroom': (4.9, 4.9),
         'living': (2.8, 6.8)}
START_HEADING = 90
TRACE = False
PIECE_CM, PIECE_DEG = 100, 45      # long moves are driven in pieces, re-planning in between
MAX_PIECES = 30
ESCAPE_CM, MAX_ESCAPES = 15, 5       # after a bump: back away 15 cm, then re-plan


def call(url, body=None, timeout=20):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data, {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'error': e.read().decode(errors='replace')}


class Rover:
    def __init__(self, drive, cam, depth, start, localizer=None):
        self.drive, self.cam, self.depth, self.start, self.loc = drive, cam, depth, start, localizer
        call(drive + '/odometer/reset', {})
        self.last_odo = (0.0, 0.0, 0.0)
        self.spread = 0.0
        self.bumps = call(drive + '/imu').get('bumps', 0)

    def odometry(self):
        """World pose from the odometer alone (its frame starts at the known start pose)."""
        o = call(self.drive + '/status')['odometer']
        x0, y0, h0 = self.start
        c, s = math.cos(math.radians(h0)), math.sin(math.radians(h0))
        return (x0 + o['x_cm'] / 100 * c - o['y_cm'] / 100 * s, y0 + o['x_cm'] / 100 * s + o['y_cm'] / 100 * c,
                h0 + o['heading_deg']), (o['x_cm'] / 100, o['y_cm'] / 100, o['heading_deg'])

    def estimate(self):
        world, odo = self.odometry()
        if not self.loc:
            return world
        # the step since last time, in the rover's own frame, moves every guess
        dx, dy = odo[0] - self.last_odo[0], odo[1] - self.last_odo[1]
        dh = (odo[2] - self.last_odo[2] + 180) % 360 - 180          # the odometer's heading wraps at ±180
        h = math.radians(self.last_odo[2])
        bumps = call(self.drive + '/imu').get('bumps', 0)
        self.loc.predict(dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h), dh,
                         bumped=bumps > self.bumps)
        self.bumps = bumps
        self.last_odo = odo
        seen = call(self.depth + '/status?columns=16&realistic=1')['columns']
        self.loc.update(seen)
        pose, self.spread = self.loc.estimate()
        if TRACE:
            t = call(self.cam + '/truth')
            print(f'      truth ({t["x_m"]:.2f},{t["y_m"]:.2f},{t["heading_deg"]:.0f})  est ({pose[0]:.2f},{pose[1]:.2f},'
                  f'{pose[2]:.0f}) spread {self.spread:.2f}  odo ({world[0]:.2f},{world[1]:.2f},{world[2]:.0f})', flush=True)
        return pose

    def truth(self):
        t = call(self.cam + '/truth')
        return t['x_m'], t['y_m'], t['heading_deg'], t['collisions'], t.get('barrier_hits', 0)


def piece(move):
    kind, amount = move
    return kind, min(amount, PIECE_CM if kind in ('forward', 'back') else PIECE_DEG)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rooms', default='kitchen,bathroom,bedroom,living,home')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--port', type=int, default=9900)
    ap.add_argument('--speed-error', default='0', help='the sim rover drives this much faster than calibrated (0.1 = 10%%)')
    ap.add_argument('--gyro-drift', default='0.02', help='gyro bias spread, deg/s')
    ap.add_argument('--localize', action='store_true', help='correct the odometer against the map (localizer.py)')
    ap.add_argument('--trace', action='store_true', help='print truth / estimate / odometry after every piece')
    args = ap.parse_args()
    global TRACE
    TRACE = args.trace
    apt = json.loads((HERE / 'apartment/apartment.json').read_text())
    rects, size = apt['walls'], apt['size_m']
    start = (*ROOMS['home'], START_HEADING)
    drive, cam, depth = (f'http://127.0.0.1:{args.port + k}' for k in (0, 1, 2))
    sim = subprocess.Popen([sys.executable, str(HERE / 'apartment_sim.py'), '--no-camera', '--seed', str(args.seed),
                            '--start', ','.join(map(str, start)), '--drive-port', str(args.port),
                            '--camera-port', str(args.port + 1), '--exact-depth-port', str(args.port + 2),
                            '--speed-error', args.speed_error, '--gyro-drift', args.gyro_drift],
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    done = threading.Event()
    try:
        for _ in range(50):
            try:
                call(depth + '/status')
                break
            except OSError:
                time.sleep(0.2)
        seen_by_sensor = [r for r in rects if r.get('kind') != 'barrier']      # virtual doors are invisible
        rover = Rover(drive, cam, depth, start, Localizer(seen_by_sensor, size, start, seed=args.seed)
                      if args.localize else None)
        call(drive + '/arm', {'armed': True, 'fresh': True})

        def lease():
            while not done.is_set():
                call(drive + '/arm', {'armed': True, 'renew': True}, timeout=2)
                done.wait(0.5)
        threading.Thread(target=lease, daemon=True).start()
        print(f'{"leg":>16} {"pieces":>6} {"min":>5} {"thinks off":>10} {"really off":>10} {"hits":>4}  result')
        for room in args.rooms.split(','):
            goal, t0, pieces, escapes, result = ROOMS[room], time.monotonic(), 0, 0, 'arrived'
            _, _, _, hits0, doors0 = rover.truth()
            while True:
                est = rover.estimate()
                if math.hypot(est[0] - goal[0], est[1] - goal[1]) <= P.GOAL_M:
                    break
                if pieces >= MAX_PIECES:
                    result = 'gave up (too many pieces)'
                    break
                moves, info = P.plan(rects, size, est, goal, strict_start=False)
                if moves is None:
                    result = f'no route: {info}'
                    break
                kind, amount = piece(moves[0])
                bumps = call(drive + '/imu').get('bumps', 0)
                out = call(drive + '/move', {'kind': kind, 'amount': amount}, timeout=30)
                pieces += 1
                if out.get('error') or out.get('aborted'):
                    result = f'move failed: {out.get("error") or "aborted"}'
                    break
                if call(drive + '/imu').get('bumps', 0) > bumps:      # hit something: back away first
                    escapes += 1
                    if escapes > MAX_ESCAPES:
                        result = f'gave up ({escapes - 1} bumps on this leg)'
                        break
                    away = 'back' if kind in ('forward', 'left', 'right') else 'forward'
                    call(drive + '/move', {'kind': away, 'amount': ESCAPE_CM}, timeout=30)
            est, (tx, ty, th, hits, doors) = rover.estimate(), rover.truth()
            print(f'{"-> " + room:>16} {pieces:>6} {(time.monotonic() - t0) / 60:>5.1f} '
                  f'{math.hypot(est[0] - goal[0], est[1] - goal[1]) * 100:>8.0f}cm '
                  f'{math.hypot(tx - goal[0], ty - goal[1]) * 100:>8.0f}cm {hits - hits0:>4}  {result}', flush=True)
    finally:
        done.set()
        try:
            call(drive + '/arm', {'armed': False})
        except OSError:
            pass
        sim.terminate()
        sim.wait(5)


if __name__ == '__main__':
    main()
