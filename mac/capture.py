"""Photo capture for rebuilding the apartment later (photogrammetry / Gaussian splatting -> a
simulation scene). While recording is on and the rover is moving (a move, a pulse or manual
driving), saves CAPTURE_FPS frames a second, each with a JSON sidecar: time, frame number and age,
the drive server's odometer (x, y, heading from drive time), the heading stick's reading, and the
camera depth service's obstacle reading when it's running.

Files: mac/captures/<date>/<time>_<frame>.jpg + .json (gitignored, local only). Recording stops by
itself when the disk has less than MIN_FREE_GB left.
"""
import datetime
import json
import shutil
import threading
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CAPTURE_DIR = ROOT / 'captures'
CAPTURE_FPS = 2.0
MIN_FREE_GB = 1.0


def read_json(url, timeout=1.0):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.load(response)


class Recorder:
    def __init__(self, link, drive_url, depth_url, folder=CAPTURE_DIR, free_bytes=None):
        self.link, self.drive_url, self.depth_url = link, drive_url, depth_url
        self.folder = Path(folder)
        self.free_bytes = free_bytes or (lambda: shutil.disk_usage(self.folder.parent).free)
        self.on = False
        self.saved = 0
        self.last_seq = None
        self.why_stopped = None
        self.last_path = None

    def set(self, on):
        self.on = bool(on)
        if self.on:
            self.why_stopped = None
        return self.status()

    def status(self):
        return {'on': self.on, 'saved': self.saved, 'folder': str(self.folder), 'why_stopped': self.why_stopped,
                'last': self.last_path.name if self.last_path else None, 'fps': CAPTURE_FPS}

    def moving(self, drive):
        return bool(drive.get('busy') or drive.get('holding'))

    def step(self):
        """One capture opportunity; returns the saved image path or None."""
        if not self.on:
            return None
        if self.free_bytes() < MIN_FREE_GB * 1e9:
            self.on, self.why_stopped = False, f'disk has under {MIN_FREE_GB:.0f} GB free'
            return None
        try:
            drive = read_json(self.drive_url + '/status')
        except (OSError, ValueError):
            return None
        if not self.moving(drive):
            return None
        frame = self.link.latest()
        if frame is None or frame.seq == self.last_seq:
            return None
        self.last_seq = frame.seq
        meta = {'time': datetime.datetime.now().isoformat(timespec='milliseconds'), 'frame_seq': frame.seq,
                'frame_age_ms': None if frame.age() is None else round(frame.age() * 1000),
                'odometer': drive.get('odometer'), 'imu': drive.get('imu'), 'last': drive.get('last')}
        try:
            depth = read_json(self.depth_url + '/status', timeout=0.3)
            meta['depth'] = {k: depth.get(k) for k in ('sectors', 'raw', 'horizon_row')} if depth.get('ok') else None
        except (OSError, ValueError):
            meta['depth'] = None
        now = datetime.datetime.now()
        day = self.folder / f'{now:%Y-%m-%d}'
        day.mkdir(parents=True, exist_ok=True)
        path = day / f'{now:%H%M%S}_{now.microsecond // 1000:03d}_{frame.seq}.jpg'
        path.write_bytes(frame.jpeg)
        path.with_suffix('.json').write_text(json.dumps(meta))
        self.saved += 1
        self.last_path = path
        return path

    def run(self):
        while True:
            try:
                self.step()
            except Exception as error:          # keep recording through odd frames
                self.why_stopped = f'{type(error).__name__}: {error}'
            time.sleep(1 / CAPTURE_FPS)

    def start(self):
        threading.Thread(target=self.run, daemon=True).start()
        return self
