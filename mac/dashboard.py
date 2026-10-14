"""Rover camera dashboard: live XIAO view, link health, and the exact planner view.

Run: python3 mac/dashboard.py [--host rover-cam.local] [--port 8010]
Then open http://localhost:8010/.

"Observe" waits for a fresh frame captured after the click, saves it to mac/vision-output/,
and runs the ported Mini-T detector on it. The planner pane shows that labeled frame with its
age, so what you see is exactly what a planner would act on. No drive commands are sent.
"""
import argparse
import json
import os
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from xiao_link import DEFAULT_HOST, MAX_FRAME_AGE_S, StaleFrame, XiaoLink

ROOT = Path(__file__).resolve().parent
FRAMES = ROOT / 'vision-output'
MINI_T = Path.home() / 'Documents/Projects/ai-driven-mini-t'
DETECTOR_URL = 'http://127.0.0.1:8767'

_detector_lock = threading.Lock()


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


class Dashboard:
    def __init__(self, link):
        self.link = link
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


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>
<title>Rover camera</title>
<style>
:root{--bg:#101214;--panel:#1a1d21;--fg:#d8dde3;--dim:#8a929b;--ok:#3fb950;--bad:#f85149}
body{margin:0;background:var(--bg);color:var(--fg);font:14px -apple-system,system-ui,sans-serif}
main{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
section{flex:1 1 420px;background:var(--panel);border-radius:10px;padding:12px;min-width:0}
h2{font-size:13px;margin:0 0 8px;color:var(--dim);text-transform:uppercase;letter-spacing:.05em}
img{width:100%;display:block;border-radius:6px;background:#000;aspect-ratio:4/3;object-fit:contain}
#state{font-weight:600}#state.ok{color:var(--ok)}#state.stale{color:var(--bad)}
dl{display:grid;grid-template-columns:auto 1fr;gap:2px 12px;margin:8px 0 0;font:12px ui-monospace,monospace}
dt{color:var(--dim)}dd{margin:0}
form{display:flex;gap:8px;margin-bottom:8px}input{flex:1;background:#0b0d0f;color:var(--fg);border:1px solid #333;border-radius:6px;padding:6px}
button{background:#2f81f7;color:#fff;border:0;border-radius:6px;padding:6px 14px;font-weight:600}
pre{font:11px ui-monospace,monospace;white-space:pre-wrap;color:var(--dim);margin:8px 0 0}
</style>
<main>
<section><h2>Live · <span id=state>connecting</span></h2><img id=live alt="live view">
<dl id=health></dl></section>
<section><h2>Planner view (exact detector input)</h2>
<form id=f><input id=goal placeholder="detector prompt (optional; no Phase 1 target)" aria-label="detector prompt"><button>Observe</button></form>
<img id=planner alt="planner view"><dl id=obs></dl><pre id=raw></pre></section>
</main>
<script>
const $=id=>document.getElementById(id);
const row=(k,v)=>`<dt>${k}</dt><dd>${v??'–'}</dd>`;
function nextLive(){$('live').src='/live.jpg?'+Date.now()}
$('live').onload=()=>setTimeout(nextLive,30);$('live').onerror=()=>setTimeout(nextLive,500);nextLive();
async function poll(){try{const h=await (await fetch('/status')).json();const t=h.telemetry||{};
$('state').textContent=h.state+(h.problems.length?' — '+h.problems.join('; '):'');$('state').className=h.state;
$('health').innerHTML=row('frame age',h.frame_age_ms+' ms (limit '+h.max_frame_age_ms+')')+row('fps (Mac)',h.fps)+
row('telemetry age',h.telemetry_age_ms+' ms')+row('rssi',t.rssi+' dBm')+row('mic',t.mic_dbfs+' dBFS')+
row('fw',t.fw)+row('reset reason',t.reset_reason)+row('device restarts seen',h.device_restarts_seen)+
row('errors',JSON.stringify(h.errors))+row('last error',h.last_error);}catch(e){$('state').textContent='dashboard offline';$('state').className='stale'}
setTimeout(poll,250)}poll();
$('f').onsubmit=async e=>{e.preventDefault();$('obs').innerHTML=row('status','observing…');
const r=await fetch('/observe',{method:'POST',body:JSON.stringify({goal:$('goal').value})});const o=await r.json();
if(!r.ok){$('obs').innerHTML=row('refused',o.error);return}
$('planner').src='/planner.jpg?'+Date.now();
$('obs').innerHTML=row('decision',o.decision)+row('frame seq',o.seq)+row('frame age at capture',o.frame_age_ms_at_capture+' ms')+
row('wait for fresh frame',o.capture_wait_ms+' ms')+row('total',o.total_ms+' ms')+row('detector error',o.detector_error);
$('raw').textContent=o.measurement?JSON.stringify(o.measurement,null,1):''};
</script>"""


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
                return self.reply(200, PAGE.encode(), 'text/html; charset=utf-8')
            if path == '/status':
                health = dashboard.link.health()
                health['observation'] = dashboard.observation
                return self.reply(200, health)
            if path == '/live.jpg':
                frame = dashboard.link.latest()
                if frame is None:
                    return self.reply(503, {'error': 'no frames yet'})
                age = frame.age()
                return self.reply(200, frame.jpeg, 'image/jpeg',
                                  {'X-Frame-Seq': str(frame.seq),
                                   'X-Frame-Age-Ms': 'unknown' if age is None else str(round(age * 1000))})
            if path == '/planner.jpg':
                if dashboard.planner_image is None:
                    return self.reply(404, {'error': 'no observation yet'})
                return self.reply(200, dashboard.planner_image, 'image/jpeg')
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
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
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(Dashboard(link)))
    print(f'Rover dashboard on http://localhost:{args.port}/  (camera {args.host})', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
