"""Shadow mode: record what the camera saw, what you did and what the explorer would have done, while
you drive from the dashboard; then tune the explorer to drive more like you.

Recording (Shadow switch on the dashboard): while you hold a drive key, SHADOW_FPS samples a second go
to mac/shadow/<session>/ (local, gitignored): fNNNNN.jpg + one line in samples.jsonl with the time,
your key, the gyro heading / turn rate / bumps, the odometer, the depth service's left/centre/right
free floor and what it sees (dog, cat, person), and the explorer's choice for that view.

Report (GET /shadow/report): how often you and the explorer agree, where you disagree (with frames),
and explorer thresholds (FORWARD_CM, TURN_CM, SIDE_CM) that would have agreed with you most, searched
over a grid with safety floors. Apply writes them to mac/explore_params.json, which explore.py
reads at start. Only the handheld remote bypasses this: its commands never reach the Mac.
"""
import datetime
import json
import shutil
import threading
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SHADOW_DIR = ROOT / 'shadow'
PARAMS_FILE = ROOT / 'explore_params.json'
SHADOW_FPS = 8.0
MIN_FREE_GB = 1.0
# what a held key means, as the explorer's actions
ACTION = {'W': 'forward', 'A': 'left', 'D': 'right', 'S': 'back', 'Z': 'back', 'C': 'back',
          'L': 'steer', 'R': 'steer'}
# safety floors for tuning: never drive forward with less than this ahead, never turn with less
FLOORS = {'FORWARD_CM': 60, 'TURN_CM': 35, 'SIDE_CM': 60}
GRID = {'FORWARD_CM': range(60, 181, 20), 'TURN_CM': range(35, 116, 10), 'SIDE_CM': range(60, 221, 20)}


def read_json(url, timeout=1.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def explorer_action(sec, p):
    """What explore.choose would do with these thresholds, as one of forward / left / right / back."""
    if sec['centre'] >= p['FORWARD_CM']:
        return 'forward'
    if sec['centre'] >= p['TURN_CM'] and max(sec['left'], sec['right']) >= p['SIDE_CM']:
        return 'left' if sec['left'] >= sec['right'] else 'right'
    return 'back'                       # a multi-point turn: its first move backs up


def current_params():
    import explore
    p = {k: getattr(explore, k) for k in FLOORS}
    if PARAMS_FILE.exists():
        p.update({k: v for k, v in json.loads(PARAMS_FILE.read_text()).items() if k in FLOORS})
    return p


class Shadow:
    def __init__(self, link, drive_url, depth_url, folder=SHADOW_DIR, free_bytes=None):
        self.link, self.drive_url, self.depth_url, self.folder = link, drive_url, depth_url, Path(folder)
        self.free_bytes = free_bytes or (lambda: shutil.disk_usage(self.folder.parent).free)
        self.on, self.session, self.samples, self.why_stopped = False, None, 0, None
        self.last_seq = None

    def set(self, on):
        if on and not self.on:
            self.session = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
            (self.folder / self.session).mkdir(parents=True, exist_ok=True)
            self.samples, self.why_stopped = 0, None
        self.on = bool(on)
        return self.status()

    def status(self):
        return {'on': self.on, 'session': self.session, 'samples': self.samples, 'why_stopped': self.why_stopped,
                'sessions': len(self.sessions())}

    def sessions(self):
        return sorted(p.name for p in self.folder.glob('*') if (p / 'samples.jsonl').exists()) if self.folder.exists() else []

    def step(self):
        """One sample if recording and you're holding a drive key; returns the sample or None."""
        if not self.on:
            return None
        if self.free_bytes() < MIN_FREE_GB * 1e9:
            self.on, self.why_stopped = False, f'disk has under {MIN_FREE_GB:.0f} GB free'
            return None
        try:
            drive = read_json(self.drive_url + '/status')
        except (OSError, ValueError):
            return None
        key = drive.get('holding')
        if not key:
            return None
        frame = self.link.latest()
        if frame is None or frame.seq == self.last_seq:
            return None
        self.last_seq = frame.seq
        try:
            depth = read_json(self.depth_url + '/status', timeout=0.3)
        except (OSError, ValueError):
            depth = {}
        sec = {k: v['cm'] for k, v in (depth.get('sectors') or {}).items()} if depth.get('ok') else None
        imu = drive.get('imu') or {}
        sample = {'i': self.samples, 't': round(time.time(), 3), 'key': key, 'action': ACTION.get(key, key),
                  'heading': imu.get('heading_deg'), 'rate': imu.get('rate_dps'), 'bumps': imu.get('bumps'),
                  'odometer': drive.get('odometer'), 'sectors': sec, 'stop_for': depth.get('stop_for'),
                  'explorer': explorer_action(sec, current_params()) if sec else None,
                  'frame_age_ms': None if frame.age() is None else round(frame.age() * 1000)}
        folder = self.folder / self.session
        (folder / f'f{self.samples:05d}.jpg').write_bytes(frame.jpeg)
        with (folder / 'samples.jsonl').open('a') as f:
            f.write(json.dumps(sample) + '\n')
        self.samples += 1
        return sample

    def run(self):
        while True:
            try:
                self.step()
            except Exception as error:
                self.why_stopped = f'{type(error).__name__}: {error}'
            time.sleep(1 / SHADOW_FPS)

    def start(self):
        threading.Thread(target=self.run, daemon=True).start()
        return self

    # ---- learning from it ----
    def load(self, sessions=None):
        out = []
        for name in sessions or self.sessions():
            path = self.folder / name / 'samples.jsonl'
            if path.exists():
                for line in path.read_text().splitlines():
                    s = json.loads(line)
                    if s.get('sectors') and s['action'] in ('forward', 'left', 'right', 'back') and not s.get('stop_for'):
                        out.append(dict(s, session=name))
        return out

    def report(self):
        samples = self.load()
        params = current_params()
        agree = lambda p: sum(explorer_action(s['sectors'], p) == s['action'] for s in samples)
        now = agree(params)
        best, best_n = dict(params), now
        for fwd in GRID['FORWARD_CM']:
            for turn in GRID['TURN_CM']:
                if turn >= fwd:
                    continue
                for side in GRID['SIDE_CM']:
                    p = {'FORWARD_CM': fwd, 'TURN_CM': turn, 'SIDE_CM': side}
                    n = agree(p)
                    if n > best_n:
                        best, best_n = p, n
        moving = [s for s in samples if s['action'] in ('forward', 'left', 'right')]
        disagree = [s for s in samples if explorer_action(s['sectors'], params) != s['action']]
        # the riskiest disagreements first: it would drive forward where you didn't, then nearest views
        disagree.sort(key=lambda s: (explorer_action(s['sectors'], params) != 'forward', s['sectors']['centre']))
        actions = ('forward', 'left', 'right', 'back')
        confusion = {you: {it: sum(1 for s in samples if s['action'] == you and explorer_action(s['sectors'], params) == it)
                           for it in actions} for you in actions}
        return {'samples': len(samples), 'moving': len(moving), 'sessions': self.sessions(),
                'params': params, 'agree': now, 'agree_pct': round(100 * now / len(samples), 1) if samples else None,
                'suggested': best, 'suggested_agree_pct': round(100 * best_n / len(samples), 1) if samples else None,
                'confusion': confusion,
                'disagreements': [{'session': s['session'], 'i': s['i'], 'you': s['action'],
                                   'explorer': explorer_action(s['sectors'], params), 'sectors': s['sectors']}
                                  for s in disagree[:24]]}


def apply_params(params):
    clean = {}
    for k, floor in FLOORS.items():
        v = int(params[k])
        if v < floor:
            raise ValueError(f'{k} below the safety floor {floor}')
        clean[k] = v
    if clean['TURN_CM'] >= clean['FORWARD_CM']:
        raise ValueError('TURN_CM must be below FORWARD_CM')
    PARAMS_FILE.write_text(json.dumps(dict(clean, applied=datetime.datetime.now().isoformat(timespec='seconds')),
                                      indent=1))
    return current_params()
