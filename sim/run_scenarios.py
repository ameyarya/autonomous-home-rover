"""Closed-loop runs of the real dog-companion code (mac/dog_behavior.py + mac/drive_client.py)
against the simulated rover and dog.

Run: sim/.venv/bin/python sim/run_scenarios.py [--seeds 6] [--only NAME] [--out sim/out/results.json]

Each step mirrors mac/dog_watcher.py: 5 Hz frames, simulated YOLO box -> Dog(x, y, height,
pixel_change from in-box grayscale difference at 160x120), Behavior.update, DriveExecutor.act;
after a pulse the motion window and pixel baseline are cleared and the next frame must be
captured after the stop. The vision model is an oracle with the watcher's cadence and latency.
"""
import argparse
import json
import multiprocessing
import math
import random
import statistics
import sys
import time as real_time
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'mac'))

import numpy as np  # noqa: E402

import drive_client  # noqa: E402
import rover_sim as rs  # noqa: E402
from dog_behavior import Behavior, Dog, Settings, ViewChange  # noqa: E402

FRAME_PERIOD_S = 0.2
CAMERA_FRAME_S = 1 / 25
DESCRIBE_EVERY_S, DESCRIBE_MIN_GAP_S, DESCRIBE_LATENCY_S = 60.0, 15.0, 8.0
MODEL_STATE_MAX_AGE_S = 600.0


class Clock:
    def __init__(self, world):
        self.world = world

    def monotonic(self):
        return self.world.t


class Oracle:
    """Stands in for Qwen: the dog's true state, `latency` after the request, sometimes unclear."""

    def __init__(self, rng):
        self.rng = rng
        self.pending = None
        self.latest = None
        self.last_started = -1e9

    def maybe_start(self, now, truth):
        if self.pending or now - self.last_started < DESCRIBE_MIN_GAP_S:
            return
        self.last_started = now
        state = 'unclear' if self.rng.random() < 0.1 else truth
        self.pending = (now + DESCRIBE_LATENCY_S, state)

    def state(self, now):
        if self.pending and now >= self.pending[0]:
            self.latest = (self.pending[0], self.pending[1])
            self.pending = None
        if self.latest and now - self.latest[0] <= MODEL_STATE_MAX_AGE_S:
            return self.latest[1]
        return None


class Episode:
    def __init__(self, world, settings, seed):
        self.w = world
        self.behavior = Behavior(settings)
        self.oracle = Oracle(random.Random(seed + 1000))
        drive_client.time = types.SimpleNamespace(monotonic=Clock(world).monotonic)
        self.executor = drive_client.DriveExecutor(post=self._post, get=lambda path, timeout=0: {'armed': True})
        self.prev_gray = None
        self.visible = False
        self.moving = None
        self.trace = []
        self.pulses = []
        self.pre_move_gray = None
        self.last_gray = None
        self.view_change = ViewChange()
        self.last_key = None
        self.move_changes = []   # (frame change after a pulse, truly blocked)

    def _post(self, path, body=None, timeout=0):
        if path == '/pulse':
            self.pulses.append((round(self.w.t, 2), body['key'], body['ms']))
            self.last_key = body['key']
            self.w.pulse(body['key'], body['ms'])
        return {'ok': True}

    def step(self):
        w = self.w
        now = w.t
        gray, mask, _ = w.camera()
        if self.pre_move_gray is not None:
            change = float(np.mean(np.abs(gray - self.pre_move_gray)))
            self.move_changes.append((change, w.last_pulse_blocked, self.last_key))
            self.behavior.note_move(self.view_change.blocked(self.last_key, change), self.last_key)
            self.pre_move_gray = None
        elif self.last_gray is not None:
            self.view_change.note_still(float(np.mean(np.abs(gray - self.last_gray))))
        self.last_gray = gray
        found = w.detect(mask)
        dog = None
        if found:
            x1, y1, x2, y2 = found['box']
            pc = None
            if self.prev_gray is not None:
                a, b, c, d = int(x1 * 160), int(y1 * 120), int(x2 * 160), int(y2 * 120)
                if c - a >= 4 and d - b >= 4:
                    pc = float(np.mean(np.abs(gray[b:d, a:c] - self.prev_gray[b:d, a:c])))
            dog = Dog(x=(x1 + x2) / 2 * 100, y=(y1 + y2) / 2 * 100, height=(y2 - y1) * 100,
                      confidence=found['confidence'], pixel_change=pc)
        self.prev_gray = gray
        # Description cadence (mirrors dog_watcher._track_visibility).
        truth = w.dog.activity()
        if dog and not self.visible:
            self.oracle.maybe_start(now, truth)
        self.visible = dog is not None or (self.visible and now - getattr(self, 'last_det', now) <= 3)
        if dog:
            self.last_det = now
            m = self.behavior.motion.moving
            if self.moving is not None and m != self.moving:
                self.oracle.maybe_start(now, truth)
            self.moving = m
            if now - self.oracle.last_started >= DESCRIBE_EVERY_S:
                self.oracle.maybe_start(now, truth)
        decision = self.behavior.update(now, dog, self.oracle.state(now), camera_ok=True, rover_moving=False)
        result = self.executor.act(decision.action, now)
        sent = result.get('sent')
        if sent and sent != 'X':
            self.behavior.motion.samples.clear()
            self.prev_gray = None
            self.pre_move_gray = gray
        in_view = bool(mask.sum() >= 40)
        self.trace.append({'t': round(now, 2), 'state': decision.state, 'action': decision.action, 'sent': sent,
                           'reason': decision.reason,
                           'detected': dog is not None, 'in_view': in_view, 'gap': round(w.gap_to_dog(), 3),
                           'dog': truth, 'dog_speed': w.dog.speed, 'x': round(w.x, 3), 'y': round(w.y, 3),
                           'yaw': round(w.yaw, 3)})
        return sent

    def run(self, duration):
        w = self.w
        while w.t < duration:
            started = w.t
            sent = self.step()
            if sent and sent != 'X':
                w.advance(CAMERA_FRAME_S)              # next frame captured after the stop
            else:
                w.advance(max(CAMERA_FRAME_S, started + FRAME_PERIOD_S - w.t))


# ---- scenarios ----
# rover pose (rear axle x, y, yaw); dog (x, y, yaw); script built per seed.

def _jitter(rng, v, d):
    return v + rng.uniform(-d, d)


def walk_then_sleep(rng):
    pts = [(rng.uniform(1.0, 4.2), rng.uniform(0.9, 2.9)) for _ in range(4)]
    script = []
    for p in pts:
        script += [('walk', p, rng.uniform(0.35, 0.8)), ('stand', rng.uniform(2, 8), rng.random() < 0.5)]
    script += [('lie', 60), ('sleep', 900)]
    return (0.6, _jitter(rng, 2.0, 0.3), 0.0), (2.0, 2.0, math.pi), script, 720


def hide_to_side(rng):
    side = rng.choice([-1, 1])
    target = (1.3, 2.0 + side * 1.4)            # beside/behind the rover, outside the camera view
    # Dashes off to the side faster than the rover can turn, then lies down out of view.
    script = [('stand', 4, True), ('run', (2.0, 2.0 + side * 1.2), 1.5), ('run', target, 1.5), ('lie', 600)]
    return (1.0, 2.0, 0.0), (2.3, 2.0, math.pi), script, 150


def leave_room(rng):
    script = [('stand', 4, False), ('walk', (4.6, 2.0), 0.6), ('walk', (5.5, 2.0), 0.6), ('walk', (6.6, 2.6), 0.6),
              ('lie', 600)]
    return (1.2, _jitter(rng, 2.0, 0.3), 0.0), (2.6, 2.0, 0.0), script, 150


def charge(rng):
    def runs_at(w):
        return lambda: rs.rover_corners(w.x, w.y, w.yaw, w.p)[1]
    script = [('stand', 3, True), ('approach_rover', 1.4, 0.05), ('stand', 6, True), ('walk', (3.5, 2.6), 0.5),
              ('stand', 20, False)]
    return (1.0, 2.0, 0.0), (2.6, _jitter(rng, 2.0, 0.2), math.pi), script, 60


def close_off_centre(rng):
    side = rng.choice([-1, 1])
    script = [('lie', 6), ('stand', 2, True), ('walk', (_jitter(rng, 2.6, 0.3), 2.0 + side * 0.9), 0.4),
              ('stand', 30, False)]
    return (1.0, 2.0, 0.0), (1.55, 2.0 + side * 0.32, math.pi + side * 0.6), script, 70


def wake_from_sleep(rng):
    script = [('sleep', 25), ('stand', 3, True), ('walk', (_jitter(rng, 3.8, 0.3), _jitter(rng, 1.3, 0.3)), 0.5),
              ('walk', (_jitter(rng, 2.5, 0.3), 3.0), 0.5), ('stand', 30, False)]
    return (0.8, 2.0, 0.0), (2.0, 2.0, math.pi / 2), script, 110


SCENARIOS = {
    'walk_then_sleep': walk_then_sleep,
    'hide_to_side': hide_to_side,
    'leave_room': leave_room,
    'charge': charge,
    'close_off_centre': close_off_centre,
    'wake_from_sleep': wake_from_sleep,
}


def build(name, seed, params, settings):
    rng = random.Random(seed)
    rover, (dx, dy, dyaw), script, duration = SCENARIOS[name](rng)
    world = rs.World(params, seed=seed)
    dog = rs.Dog(dx, dy, dyaw, [], seed=seed)
    resolved = []
    for task in script:
        if task[0] == 'approach_rover':
            resolved.append(('approach', lambda w=world: ((rs.rover_corners(w.x, w.y, w.yaw, w.p)[1][0] + rs.rover_corners(w.x, w.y, w.yaw, w.p)[2][0]) / 2,
                                                           (rs.rover_corners(w.x, w.y, w.yaw, w.p)[1][1] + rs.rover_corners(w.x, w.y, w.yaw, w.p)[2][1]) / 2),
                             task[1], task[2]))
        else:
            resolved.append(task)
    dog.script = resolved
    world.place(rover, dog)
    return world, duration


def metrics(name, ep, world):
    tr = ep.trace
    following = [r for r in tr if r['state'] == 'following']
    states = [r['state'] for r in tr]
    first = lambda pred: next((r['t'] for r in tr if pred(r)), None)
    out = {
        'scenario': name,
        'duration_s': round(world.t, 1),
        'dog_contacts': world.dog_contacts,
        'bumps': world.bumps,
        'min_gap_m': round(min(r['gap'] for r in tr), 3),
        'pulses': len(ep.pulses),
        'distance_m': round(world.odometer, 2),
        'in_view_while_following': round(sum(r['in_view'] for r in following) / len(following), 3) if following else None,
        'final_state': states[-1],
        'time_following_s': round(len(following) * FRAME_PERIOD_S, 1),
        'searched': 'searching' in states,
        'blocked_pulses': sum(b for _, b, _k in ep.move_changes),
        'stuck_stops': sum(1 for r in tr if r['action'] == 'park' and r['reason'].startswith('stuck')),
        'move_changes': [(round(c, 2), b, k) for c, b, k in ep.move_changes],
    }
    if name == 'walk_then_sleep':
        slept = first(lambda r: r['dog'] == 'sleeping')
        parked = next((r['t'] for r in tr if slept and r['t'] >= slept and r['action'] == 'park'), None)
        out['park_after_sleep_s'] = round(parked - slept, 1) if parked else None
        still = [r['gap'] for r in following if r['dog'] != 'active' and r['in_view']]
        out['standoff_gap_m'] = round(statistics.median(still), 2) if still else None
    if name in ('hide_to_side', 'leave_room'):
        lost = first(lambda r: r['state'] == 'searching')
        found = next((r['t'] for r in tr if lost and r['t'] > lost and r['state'] == 'following'), None)
        out['found_after_s'] = round(found - lost, 1) if found else None
        out['parked_after_search'] = any(r['action'] == 'park' and lost and r['t'] > lost for r in tr)
    if name == 'wake_from_sleep':
        woke = first(lambda r: r['dog'] == 'active')
        follow = next((r['t'] for r in tr if woke and r['t'] >= woke and r['state'] == 'following'), None)
        out['follow_after_wake_s'] = round(follow - woke, 1) if follow else None
    if name == 'charge':
        out['backed_off'] = any(r['action'] == 'back' for r in tr)
    return out


def scan_rotation(params):
    """Arc + reverse pairs from the drive client's scan cycle: degrees per pair and drift."""
    world = rs.World(params, render=False)
    world.place((2.5, 2.0, 0.0), rs.Dog(-5, -5, 0, [('stand', 1e9, False)]))
    cycle = drive_client.SCANS['scan_right']
    start = (world.x, world.y, world.yaw)
    for key, ms in cycle:
        world.pulse(key, ms)
    per_pair = math.degrees(abs(world.yaw - start[2]))
    pairs = 0
    world.place((2.5, 2.0, 0.0), world.dog)
    spots = []
    while abs(world.yaw) < 2 * math.pi and pairs < 200:
        for key, ms in cycle:
            world.pulse(key, ms)
        pairs += 1
        spots.append((world.x, world.y))
    xs, ys = [s[0] for s in spots], [s[1] for s in spots]
    observe = drive_client.OBSERVE_S + FRAME_PERIOD_S
    seconds = sum((ms / 1000 + params.ack_s + observe) for _, ms in cycle) * pairs
    return {'deg_per_pair': round(per_pair, 1), 'pairs_for_360': pairs, 'seconds_for_360': round(seconds, 1),
            'footprint_m': round(max(max(xs) - min(xs), max(ys) - min(ys)), 2)}


def run_one(job):
    name, seed, trace = job
    world, duration = build(name, seed, rs.RoverParams(), Settings())
    ep = Episode(world, Settings(), seed)
    ep.run(duration)
    m = metrics(name, ep, world)
    m['seed'] = seed
    if trace:
        m['trace'] = ep.trace
        m['pulse_log'] = ep.pulses
    world.close()
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seeds', type=int, default=6)
    ap.add_argument('--only', choices=list(SCENARIOS))
    ap.add_argument('--out', default=str(HERE / 'out/results.json'))
    ap.add_argument('--trace', action='store_true', help='include per-frame traces in the output')
    ap.add_argument('--jobs', type=int, default=max(1, (multiprocessing.cpu_count() or 2) - 2))
    args = ap.parse_args()
    params, settings = rs.RoverParams(), Settings()
    started = real_time.time()
    results = {'rover_params': vars(params), 'behavior_settings': {k: getattr(settings, k) for k in vars(settings)},
               'drive_pulses': drive_client.PULSES, 'scan_cycle': drive_client.SCANS,
               'scan_rotation': scan_rotation(params), 'episodes': []}
    print('scan rotation:', results['scan_rotation'], flush=True)
    jobs = [(name, seed, args.trace) for name in ([args.only] if args.only else SCENARIOS) for seed in range(args.seeds)]
    with multiprocessing.get_context('spawn').Pool(args.jobs) as pool:
        for m in pool.imap(run_one, jobs):
            results['episodes'].append(m)
            print(json.dumps({k: v for k, v in m.items() if k not in ('trace', 'pulse_log')}), flush=True)
    results['wall_time_s'] = round(real_time.time() - started, 1)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=1))
    print(f'wrote {out} in {results["wall_time_s"]} s')


if __name__ == '__main__':
    main()
