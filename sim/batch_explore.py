"""Many explorer runs in the simulated apartment, in parallel, scored from the simulator's truth.

Run: sim/.venv/bin/python sim/batch_explore.py --runs 8 --parallel 4 --steps 40
Each run: its own simulator (exact ray depth, no rendering) on its own ports, a random start on free
floor, the real mac/explore.py pointed at it. Scores per run: collisions, seconds spent pushing
against something, closest approach, distance driven, why the explorer stopped.
Results: sim/out/batch.json (gitignored) and a table.
"""
import argparse
import json
import math
import random
import subprocess
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(HERE))
import apartment_sim  # noqa: E402


HOME = (2.0, 4.5)               # a point in the living room: starts must be reachable from it


def reachable(walls, cell=0.1):
    """Cells the rover's centre can reach from HOME without crossing walls, furniture or virtual doors
    (a flood fill keeping 15 cm clear: about half the rover's width)."""
    nx, ny = int(8 / cell), int(8 / cell)
    ok = lambda i, j: walls.clearance((i + .5) * cell, (j + .5) * cell) > 0.15
    start = (int(HOME[0] / cell), int(HOME[1] / cell))
    seen, todo = {start}, [start]
    while todo:
        i, j = todo.pop()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and (a, b) not in seen and ok(a, b):
                seen.add((a, b))
                todo.append((a, b))
    return [((i + .5) * cell, (j + .5) * cell) for i, j in seen]


def free_starts(n, seed):
    walls = apartment_sim.Walls(json.loads((HERE / 'apartment/apartment.json').read_text())['walls'])
    cells = reachable(walls)
    rng, out = random.Random(seed), []
    while len(out) < n:
        (x, y), h = rng.choice(cells), rng.uniform(-180, 180)
        if walls.clearance(x, y) > 0.45 and not walls.hits(x, y, math.radians(h)):
            out.append((round(x, 2), round(y, 2), round(h)))
    return out


def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.load(r)


def run_one(i, start, steps, seed, extra=()):
    base = 9800 + i * 10
    drive, cam, depth = base, base + 1, base + 2
    (HERE / 'out').mkdir(exist_ok=True)
    log = open(HERE / f'out/sim_{i}.log', 'w')
    sim = subprocess.Popen([sys.executable, str(HERE / 'apartment_sim.py'), '--no-camera', '--seed', str(seed + i),
                            '--start', ','.join(map(str, start)), '--drive-port', str(drive),
                            '--camera-port', str(cam), '--exact-depth-port', str(depth)],
                           stdout=log, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                get(f'http://127.0.0.1:{depth}/status')
                break
            except OSError:
                time.sleep(0.2)
        t = time.monotonic()
        out = subprocess.run(['python3', str(REPO / 'mac/explore.py'), '--steps', str(steps), '--no-capture',
                              '--drive', f'http://127.0.0.1:{drive}', '--depth', f'http://127.0.0.1:{depth}', *extra],
                             capture_output=True, text=True, timeout=steps * 30)
        lines = out.stdout.strip().splitlines()
        stopped = next((l[9:] for l in lines if l.startswith('stopped:')), out.stderr.strip()[-120:] or '?')
        moves = [json.loads(l) for l in lines if l.startswith('{')]
        try:
            truth = get(f'http://127.0.0.1:{cam}/truth')
        except OSError:
            tail = (HERE / f'out/sim_{i}.log').read_text().strip().splitlines()[-1:] or ['?']
            truth = {'collisions': 0, 'barrier_hits': 0, 'pushing_s': 0, 'path_m': 0, 'min_clearance_m': 9,
                     'collision_events': []}
            stopped = f'SIMULATOR CRASHED: {tail[0][:100]}'
        print(f'run {i} done: {stopped}', flush=True)
        return {'run': i, 'start': start, 'steps': len(moves), 'stopped': stopped, 'minutes': round((time.monotonic() - t) / 60, 1),
                'turns': sum(1 for m in moves if str(m.get('move', '')).startswith('turn_')), **truth}
    finally:
        sim.terminate()
        sim.wait(5)
        log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=8)
    ap.add_argument('--parallel', type=int, default=4)
    ap.add_argument('--steps', type=int, default=40)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--reverse-arcs', action='store_true', help="explorer's old multi-point turns (reverse arcs)")
    args = ap.parse_args()
    starts = free_starts(args.runs, args.seed)
    with ThreadPoolExecutor(args.parallel) as pool:
        extra = ('--reverse-arcs',) if args.reverse_arcs else ()
        results = list(pool.map(lambda a: run_one(a[0], a[1], args.steps, args.seed, extra), enumerate(starts)))
    (HERE / 'out').mkdir(exist_ok=True)
    (HERE / 'out/batch.json').write_text(json.dumps(results, indent=1))
    print(f'{"run":>3} {"start":>18} {"steps":>5} {"turns":>5} {"path m":>6} {"hits":>4} {"doors":>5} {"push s":>6} {"closest":>7}  stopped')
    for r in results:
        print(f'{r["run"]:>3} {str(r["start"]):>18} {r["steps"]:>5} {r["turns"]:>5} {r["path_m"]:>6} {r["collisions"]:>4} '
              f'{r.get("barrier_hits", 0):>5} {r["pushing_s"]:>6} {r["min_clearance_m"]:>7}  {r["stopped"]}')
    hits = sum(r['collisions'] for r in results)
    doors = sum(r.get('barrier_hits', 0) for r in results)
    events = [dict(e, run=r['run']) for r in results for e in r.get('collision_events', [])]
    if events:
        print('\ncollisions:')
        for e in events:
            print(f"  run {e['run']}: {e['part']} hit {e['with']} while moving {e['moving']} (steer {e['steer']}) "
                  f"at ({e['x']}, {e['y']}) heading {e['heading']}")
    print(f'\n{len(results)} runs: {hits} collisions, {doors} virtual-door touches, {sum(r["path_m"] for r in results):.1f} m driven, '
          f'closest approach {min(r["min_clearance_m"] for r in results):.2f} m')


if __name__ == '__main__':
    main()
