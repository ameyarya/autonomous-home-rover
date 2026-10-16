"""Rover dashboard: live camera, the dog watcher's exact input, drive controls, camera health.

Run: python3 mac/dashboard.py [--host rover-cam.local] [--port 8010]
Then open http://localhost:8010/. The page is mac/dashboard.html.

- Services: start/stop the drive server (drive/drive_server.py) and dog watcher
  (mac/dog_watcher.py) from the header; ones started elsewhere show as running.
- Drive: Arm (lease renewed by the page), STOP (Esc/Space), hold-to-drive with WASD/arrows.
- Camera: health from XiaoLink, settings via the XIAO's /config, and an alert whenever the
  XIAO restarts (power drop on the magnetic cable, brownout, crash), with its reset reason.
- Detector check: "Observe" saves a fresh frame and runs the ported Mini-T detector on it.
"""
import argparse
import collections
import datetime
import json
import os
import subprocess
import sys
import urllib.parse
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from xiao_link import DEFAULT_HOST, MAX_FRAME_AGE_S, StaleFrame, XiaoLink

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
PAGE_FILE = ROOT / 'dashboard.html'
FRAMES = ROOT / 'vision-output'
MINI_T = Path.home() / 'Documents/Projects/ai-driven-mini-t'
DETECTOR_URL = 'http://127.0.0.1:8767'
DOG_WATCHER_URL = 'http://127.0.0.1:8768'
DRIVE_URL = 'http://127.0.0.1:8769'

_detector_lock = threading.Lock()

# name -> (status URL, command, log file)
SERVICES = {
    'drive': (DRIVE_URL + '/status', [sys.executable, '-u', str(REPO / 'drive/drive_server.py')], ROOT / 'drive-server.log'),
    'watcher': (DOG_WATCHER_URL + '/status', [sys.executable, '-u', str(ROOT / 'dog_watcher.py')], ROOT / 'dog-watcher.log'),
}


def read_json(url, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def detector_python():
    """Interpreter for rover_detector.py: ROVER_DETECTOR_PYTHON, else the Mini-T detector venv.

    The Mini-T venv's interpreter links still point at its old checkout path, so when they are
    broken, run its bundled Python 3.12 with the venv's site-packages instead.
    """
    override = os.environ.get('ROVER_DETECTOR_PYTHON')
    if override:
        return Path(override), {}
    venv_python = MINI_T / '.detector-venv/bin/python'
    if venv_python.exists():  # follows symlinks: False when the link is broken
        return venv_python, {}
    bundled = [p for p in sorted((MINI_T / 'tools/detector-python').glob('cpython-3.12*/bin/python3.12')) if p.exists()]
    if bundled:
        return bundled[0], {'PYTHONPATH': str(MINI_T / '.detector-venv/lib/python3.12/site-packages')}
    raise RuntimeError('No detector Python found; set ROVER_DETECTOR_PYTHON to a Python with ultralytics + torch')


def ensure_detector():
    """Start mac/rover_detector.py with the Mini-T detector environment if it is not running."""
    with _detector_lock:
        try:
            if read_json(DETECTOR_URL + '/health', timeout=1).get('ready') is True:
                return
        except (OSError, ValueError):
            pass
        python, extra_env = detector_python()
        env = dict(os.environ, **extra_env)
        env.setdefault('YOLO_CONFIG_DIR', str(MINI_T / 'tools/detector-config'))
        env.setdefault('XDG_CACHE_HOME', str(MINI_T / 'tools/detector-cache'))
        with (ROOT / 'detector-server.log').open('ab') as log:
            process = subprocess.Popen([str(python), '-u', str(ROOT / 'rover_detector.py')], cwd=ROOT,
                                       env=env, stdout=log, stderr=log, start_new_session=True)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Detector failed to start; see mac/detector-server.log')
            try:
                if read_json(DETECTOR_URL + '/health', timeout=1).get('ready') is True:
                    return
            except (OSError, ValueError):
                pass
            time.sleep(0.2)
        raise RuntimeError('Detector startup timed out')


class Services:
    """Starts/stops the drive server and dog watcher; reports any instance answering on its port."""

    def __init__(self, camera_host):
        self.camera_host = camera_host
        self.processes = {}
        self.lock = threading.Lock()

    def status(self):
        out = {}
        for name, (url, _, log) in SERVICES.items():
            process = self.processes.get(name)
            managed = process is not None and process.poll() is None
            try:
                read_json(url, timeout=0.5)
                running = True
            except (OSError, ValueError):
                running = False
            starting = managed and not running
            out[name] = {'running': running, 'starting': starting, 'managed': managed,
                         'exited': process.returncode if process is not None and not managed else None,
                         'log_tail': self._tail(log) if not running else None}
        return out

    @staticmethod
    def _tail(log):
        try:
            return log.read_text(errors='replace').strip().splitlines()[-1][-200:]
        except (OSError, IndexError):
            return None

    def start(self, name):
        url, command, log = SERVICES[name]
        with self.lock:
            process = self.processes.get(name)
            if process is not None and process.poll() is None:
                return
            try:
                read_json(url, timeout=0.5)
                return  # already running elsewhere
            except (OSError, ValueError):
                pass
            if name == 'watcher':
                command = command + ['--host', self.camera_host]
            with log.open('ab') as out:
                out.write(f'\n--- started from dashboard {datetime.datetime.now():%Y-%m-%d %H:%M:%S}\n'.encode())
                self.processes[name] = subprocess.Popen(command, cwd=REPO, stdout=out, stderr=out,
                                                        start_new_session=True)

    def stop(self, name):
        with self.lock:
            process = self.processes.get(name)
            if process is None or process.poll() is not None:
                return False
            process.terminate()
            try:
                process.wait(5)
            except subprocess.TimeoutExpired:
                process.kill()
            return True

    def stop_all(self):
        for name in SERVICES:
            self.stop(name)


class RestartWatch:
    """Records every XIAO restart (uptime going backwards) with its reset reason, so a power
    drop (e.g. the magnetic cable on a bump) can't go unnoticed during drive tests."""

    def __init__(self, link):
        self.link = link
        self.events = collections.deque(maxlen=20)
        self.seen = None

    def run(self):
        while True:
            health = self.link.health()
            restarts = health.get('device_restarts_seen', 0)
            reason = (health.get('telemetry') or {}).get('reset_reason')
            if self.seen is None:
                self.seen = restarts
            elif restarts > self.seen and reason:
                self.seen = restarts
                self.events.append({'at': datetime.datetime.now().strftime('%H:%M:%S'), 'reason': reason})
            time.sleep(1)


class Dashboard:
    def __init__(self, link, services=None):
        self.link = link
        self.services = services
        self.restarts = RestartWatch(link)
        threading.Thread(target=self.restarts.run, daemon=True).start()
        self.lock = threading.Lock()
        self.observation = None
        self.planner_image = None

    def observe(self, goal):
        requested = time.monotonic()
        frame = self.link.fresh_frame(captured_after=requested, timeout=2.0)
        path = self.link.save(frame, FRAMES)
        observation = {'goal': goal, 'frame': str(path), 'seq': frame.seq,
                       'frame_age_ms_at_capture': round(frame.age() * 1000, 1),
                       'capture_wait_ms': round((time.monotonic() - requested) * 1000, 1)}
        try:
            ensure_detector()
            prepared = read_json(DETECTOR_URL + '/prepare', {'goal': goal, 'frame': str(path), 'history': []}, timeout=30)
            observation.update(model_frame=prepared['model_frame'], measurement=prepared['measurement'],
                               decision=self.decision(prepared['measurement']))
            image = Path(prepared['model_frame'])
        except Exception as error:
            observation.update(detector_error=str(error), decision='detector unavailable')
            image = path
        observation['total_ms'] = round((time.monotonic() - requested) * 1000, 1)
        with self.lock:
            self.observation = observation
            self.planner_image = image.read_bytes()
        return observation

    @staticmethod
    def decision(measurement):
        if measurement.get('target_visible'):
            x = measurement['target_x']
            side = 'centred' if 45 <= x <= 55 else ('left of centre' if x < 50 else 'right of centre')
            clipped = ', clipped by edge' if measurement.get('target_clipped') else ''
            return f"target at {x:.1f}% ({side}), height {measurement['target_height']:.1f}%{clipped}"
        if measurement.get('candidate_count'):
            return f"ambiguous: {measurement['candidate_count']} candidates"
        return 'target not located'


def make_handler(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, content_type='application/json', headers=None):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                return self.reply(200, PAGE_FILE.read_bytes(), 'text/html; charset=utf-8')
            if path == '/status':
                health = dashboard.link.health()
                health['observation'] = dashboard.observation
                health['restarts'] = list(dashboard.restarts.events)
                return self.reply(200, health)
            if path == '/services':
                return self.reply(200, dashboard.services.status())
            if path == '/cam/config':
                try:
                    with urllib.request.urlopen(f'http://{dashboard.link.host}/config', timeout=3) as response:
                        return self.reply(200, response.read())
                except OSError as error:
                    return self.reply(503, {'error': f'camera: {error}'})
            if path == '/live.jpg':
                frame = dashboard.link.latest()
                if frame is None:
                    return self.reply(503, {'error': 'no frames yet'})
                age = frame.age()
                return self.reply(200, frame.jpeg, 'image/jpeg',
                                  {'X-Frame-Seq': str(frame.seq),
                                   'X-Frame-Age-Ms': 'unknown' if age is None else str(round(age * 1000))})
            if path in ('/dog/status', '/dog/frame.jpg'):
                try:
                    with urllib.request.urlopen(DOG_WATCHER_URL + path[4:], timeout=2) as response:
                        return self.reply(200, response.read(), response.headers.get('Content-Type'))
                except OSError:
                    return self.reply(503, {'error': 'dog watcher not running'})
            if path == '/drive/status':
                try:
                    with urllib.request.urlopen(DRIVE_URL + '/status', timeout=2) as response:
                        return self.reply(200, response.read())
                except OSError:
                    return self.reply(503, {'error': 'drive server not running'})
            if path == '/planner.jpg':
                if dashboard.planner_image is None:
                    return self.reply(404, {'error': 'no observation yet'})
                return self.reply(200, dashboard.planner_image, 'image/jpeg')
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            parts = self.path.strip('/').split('/')
            if len(parts) == 3 and parts[0] == 'services' and parts[1] in SERVICES and parts[2] in ('start', 'stop'):
                try:
                    getattr(dashboard.services, parts[2])(parts[1])
                    return self.reply(200, dashboard.services.status())
                except Exception as error:
                    return self.reply(500, {'error': str(error)})
            if self.path == '/cam/config':
                size = int(self.headers.get('Content-Length', 0))
                changes = json.loads(self.rfile.read(size) or b'{}')
                query = urllib.parse.urlencode({k: v for k, v in changes.items()
                                                if k in ('framesize', 'quality', 'vflip', 'hmirror', 'brightness',
                                                         'contrast', 'saturation', 'ae_level', 'led')})
                try:
                    with urllib.request.urlopen(f'http://{dashboard.link.host}/config?{query}', timeout=5) as response:
                        return self.reply(200, response.read())
                except urllib.error.HTTPError as error:
                    return self.reply(400, {'error': error.read().decode(errors='replace') or str(error)})
                except OSError as error:
                    return self.reply(503, {'error': f'camera: {error}'})
            if self.path in ('/drive/arm', '/drive/stop', '/drive/hold'):
                size = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(size) if size else b'{}'
                try:
                    req = urllib.request.Request(DRIVE_URL + self.path[6:], body, {'Content-Type': 'application/json'})
                    with urllib.request.urlopen(req, timeout=3) as response:
                        return self.reply(200, response.read())
                except urllib.error.HTTPError as error:
                    return self.reply(error.code, error.read())
                except OSError as error:
                    return self.reply(503, {'error': f'drive server: {error}'})
            if self.path != '/observe':
                return self.reply(404, {'error': 'not found'})
            try:
                size = int(self.headers.get('Content-Length', 0))
                goal = json.loads(self.rfile.read(size) or b'{}').get('goal', '').strip()
                if not goal:
                    return self.reply(400, {'error': 'goal required'})
                self.reply(200, dashboard.observe(goal))
            except StaleFrame as error:
                self.reply(409, {'error': 'stale camera: ' + str(error)})
            except Exception as error:
                self.reply(500, {'error': str(error)})

        def log_message(self, *args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=DEFAULT_HOST, help='XIAO host (default rover-cam.local)')
    parser.add_argument('--port', type=int, default=8010)
    parser.add_argument('--max-frame-age', type=float, default=MAX_FRAME_AGE_S, help='seconds')
    args = parser.parse_args()
    link = XiaoLink(args.host, max_frame_age_s=args.max_frame_age).start()
    services = Services(args.host)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(Dashboard(link, services)))
    print(f'Rover dashboard on http://localhost:{args.port}/  (camera {args.host})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        services.stop_all()   # the drive server's stop-on-exit stops the motors


if __name__ == '__main__':
    main()
