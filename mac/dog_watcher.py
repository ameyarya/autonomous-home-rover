"""Dog watcher: find the dog in XIAO frames, judge moving/still, ask the local vision model
what he is doing, log a timeline, and decide what the rover should do.

Run: python3 mac/dog_watcher.py [--host rover-cam.local] [--sleep-after 300]
Status on http://127.0.0.1:8768/status (shown in the dashboard). Re-runs itself under the
detector Python (ultralytics + torch) if needed; uses Ollama qwen3-vl:4b-instruct for
descriptions.

Drive: decisions become bounded pulses through drive/drive_server.py, but only while the
dashboard's Arm switch is on (the server refuses pulses otherwise). Disarmed, decisions are
logged as intent only. After every pulse the watcher waits for fresh post-stop frames
(move -> stop -> observe) and restarts its dog-motion window, so camera motion is not
mistaken for dog motion.
"""
import argparse
import base64
import collections
import datetime
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
except ImportError:
    from dashboard import MINI_T, detector_python
    python, extra_env = detector_python()
    env = dict(os.environ, **extra_env)
    env.setdefault('YOLO_CONFIG_DIR', str(MINI_T / 'tools/detector-config'))
    if os.environ.get('DOG_WATCHER_REEXEC'):
        raise
    env['DOG_WATCHER_REEXEC'] = '1'
    os.execve(str(python), [str(python), str(Path(__file__).resolve()), *sys.argv[1:]], env)

from dog_behavior import Behavior, Dog, Settings, ViewChange  # noqa: E402
from drive_client import DriveExecutor  # noqa: E402
from xiao_link import DEFAULT_HOST, StaleFrame, XiaoLink  # noqa: E402

LOG_DIR = ROOT / 'dog-log'
WEIGHTS = ROOT / 'weights/yolo11s.pt'
OLLAMA = 'http://127.0.0.1:11434'
VLM_MODEL = 'qwen3-vl:4b-instruct'
DOG_CLASS = 16  # COCO
MIN_CONFIDENCE = 0.35
FRAME_PERIOD_S = 0.2
DESCRIBE_EVERY_S = 60.0
DESCRIBE_MIN_GAP_S = 15.0
MODEL_STATE_MAX_AGE_S = 600.0

DESCRIBE_PROMPT = (
    'This is a frame from a home camera. The dog is inside the green box. '
    'Describe in one short sentence what the dog is doing and where (for example: sleeping curled up '
    'on the couch, lying awake on the rug, eating from his bowl, chewing a toy, walking toward the door). '
    'Then classify his state: "sleeping" only if he appears asleep (lying still, eyes closed or head down '
    'and relaxed), "resting" if lying or sitting still but awake or unclear, "active" if moving, eating, '
    'playing or alert and standing, "unclear" if you cannot tell.')
DESCRIBE_SCHEMA = {
    'type': 'object',
    'properties': {'activity': {'type': 'string'},
                   'state': {'type': 'string', 'enum': ['sleeping', 'resting', 'active', 'unclear']}},
    'required': ['activity', 'state'],
}


def now_iso():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


class Timeline:
    """Append-only JSONL log per day plus snapshots; keeps recent events in memory."""

    def __init__(self, directory):
        self.directory = Path(directory)
        (self.directory / 'snapshots').mkdir(parents=True, exist_ok=True)
        self.recent = collections.deque(maxlen=200)
        self.lock = threading.Lock()

    def log(self, event, jpeg=None, **fields):
        record = {'t': now_iso(), 'event': event, **fields}
        if jpeg is not None:
            name = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-') + event + '.jpg'
            (self.directory / 'snapshots' / name).write_bytes(jpeg)
            record['snapshot'] = 'snapshots/' + name
        with self.lock:
            self.recent.append(record)
            day = self.directory / (datetime.date.today().isoformat() + '.jsonl')
            with day.open('a') as f:
                f.write(json.dumps(record) + '\n')
        print(record['t'], event, {k: v for k, v in fields.items() if k != 'detection'}, flush=True)


def ensure_ollama():
    try:
        urllib.request.urlopen(OLLAMA + '/api/tags', timeout=2).read()
        return
    except OSError:
        pass
    subprocess.Popen(['ollama', 'serve'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    for _ in range(50):
        time.sleep(0.2)
        try:
            urllib.request.urlopen(OLLAMA + '/api/tags', timeout=2).read()
            return
        except OSError:
            pass
    raise RuntimeError('Ollama did not start')


class Describer:
    """Runs vision-model descriptions in the background, one at a time."""

    def __init__(self, timeline):
        self.timeline = timeline
        self.busy = threading.Event()
        self.last_started = 0.0
        self.latest = None  # {'activity','state','at','seconds'}
        self.error = None

    def model_state(self, now):
        if self.latest and now - self.latest['at'] <= MODEL_STATE_MAX_AGE_S:
            return self.latest['state']
        return None

    def maybe_start(self, now, image, why):
        # image must be the caller's own copy: it is encoded later on another thread
        if self.busy.is_set() or now - self.last_started < DESCRIBE_MIN_GAP_S:
            return False
        self.busy.set()
        self.last_started = now
        threading.Thread(target=self._run, args=(image, why), daemon=True).start()
        return True

    def _run(self, image, why):
        started = time.monotonic()
        try:
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            body = {'model': VLM_MODEL, 'stream': False, 'format': DESCRIBE_SCHEMA,
                    'options': {'temperature': 0},
                    'messages': [{'role': 'user', 'content': DESCRIBE_PROMPT,
                                  'images': [base64.b64encode(encoded.tobytes()).decode()]}]}
            request = urllib.request.Request(OLLAMA + '/api/chat', json.dumps(body).encode(),
                                             {'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=120) as response:
                answer = json.loads(json.load(response)['message']['content'])
            seconds = round(time.monotonic() - started, 1)
            self.latest = {'activity': answer['activity'], 'state': answer['state'],
                           'at': time.monotonic(), 'seconds': seconds}
            self.error = None
            self.timeline.log('activity', jpeg=encoded.tobytes(), activity=answer['activity'],
                              dog_state=answer['state'], trigger=why, model_seconds=seconds)
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
            self.timeline.log('describe_error', error=self.error)
        finally:
            self.busy.clear()


class Watcher:
    def __init__(self, link, settings):
        self.link = link
        self.behavior = Behavior(settings)
        self.timeline = Timeline(LOG_DIR)
        self.describer = Describer(self.timeline)
        self.executor = DriveExecutor()
        self.model = YOLO(str(WEIGHTS))
        self.prev_gray = None
        self.visible = False
        self.visible_since = None
        self.last_detection_at = None
        self.moving = None
        self.camera_ok = None
        self.decision = None
        self.detection = None
        self.annotated = None
        self.processed = 0
        self.last_gray = None
        self.pre_move_gray = None
        self.view_change = ViewChange()
        self.lock = threading.Lock()

    def detect(self, image):
        result = self.model.predict(image, device='mps', imgsz=640, classes=[DOG_CLASS],
                                    conf=MIN_CONFIDENCE, verbose=False)[0]
        if not len(result.boxes):
            return None
        best = max(result.boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = best.xyxy[0].cpu().tolist()
        return {'box': (x1, y1, x2, y2), 'confidence': round(float(best.conf[0]), 3)}

    def pixel_change(self, gray, box):
        if self.prev_gray is None:
            return None
        h, w = gray.shape
        x1, y1, x2, y2 = (int(box[0] * w), int(box[1] * h), int(box[2] * w), int(box[3] * h))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        return float(np.mean(cv2.absdiff(gray[y1:y2, x1:x2], self.prev_gray[y1:y2, x1:x2])))

    def step(self):
        now = time.monotonic()
        try:
            frame = self.link.fresh_frame(captured_after=self.executor.captured_after, timeout=1.0)
        except StaleFrame as error:
            if self.camera_ok is not False:
                self.timeline.log('camera_stale', reason=str(error))
            self.camera_ok = False
            self._decide(now, None, camera_ok=False)
            return
        if self.camera_ok is False:
            self.timeline.log('camera_ok')
        self.camera_ok = True

        image = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
        h, w = image.shape[:2]
        gray = cv2.cvtColor(cv2.resize(image, (160, 120)), cv2.COLOR_BGR2GRAY)
        if self.pre_move_gray is not None:
            # First frame after a pulse: did the view change? If not, the move was blocked.
            change = float(np.mean(cv2.absdiff(gray, self.pre_move_gray)))
            blocked = self.view_change.blocked(self.executor.last_key, change)
            self.behavior.note_move(blocked, self.executor.last_key)
            if blocked:
                self.timeline.log('blocked', change=round(change, 2), threshold=round(self.view_change.threshold(self.executor.last_key), 2),
                                  in_a_row=self.behavior.blocked)
            self.pre_move_gray = None
        elif self.last_gray is not None:
            self.view_change.note_still(float(np.mean(cv2.absdiff(gray, self.last_gray))))   # camera noise
        self.last_gray = gray
        found = self.detect(image)
        dog = None
        if found:
            x1, y1, x2, y2 = found['box']
            norm = (x1 / w, y1 / h, x2 / w, y2 / h)
            dog = Dog(x=(x1 + x2) / 2 / w * 100, y=(y1 + y2) / 2 / h * 100, height=(y2 - y1) / h * 100,
                      confidence=found['confidence'], pixel_change=self.pixel_change(gray, norm))
            self.last_detection_at = now
        self.prev_gray = gray

        boxed = image.copy()
        if found:
            x1, y1, x2, y2 = map(int, found['box'])
            cv2.rectangle(boxed, (x1, y1), (x2, y2), (60, 220, 90), 2)
        self._track_visibility(now, dog, boxed)
        decision = self._decide(now, dog, camera_ok=True)
        self._annotate(boxed, found, decision, frame)
        self.processed += 1

    def _track_visibility(self, now, dog, boxed):
        if dog and not self.visible:
            self.visible, self.visible_since = True, now
            ok, jpeg = cv2.imencode('.jpg', boxed)
            self.timeline.log('seen', jpeg=jpeg.tobytes(), confidence=dog.confidence,
                              x=round(dog.x, 1), height=round(dog.height, 1))
            self.describer.maybe_start(now, boxed.copy(), 'seen')
        elif not dog and self.visible and now - (self.last_detection_at or now) > self.behavior.settings.lost_after_s:
            self.visible = False
            self.timeline.log('lost', visible_for_s=round(self.last_detection_at - self.visible_since, 1))
            self.moving = None
        if dog:
            moving = self.behavior.motion.moving
            if self.moving is not None and moving != self.moving:
                self.timeline.log('moving' if moving else 'still',
                                  speed=round(self.behavior.motion.speed, 2), pixel_change=dog.pixel_change)
                self.describer.maybe_start(now, boxed.copy(), 'moving' if moving else 'still')
            self.moving = moving
            if now - self.describer.last_started >= DESCRIBE_EVERY_S:
                self.describer.maybe_start(now, boxed.copy(), 'periodic')

    def _decide(self, now, dog, camera_ok):
        previous = self.decision
        decision = self.behavior.update(now, dog, self.describer.model_state(now), camera_ok=camera_ok,
                                        rover_moving=False)
        if previous is None or decision.state != previous.state:
            self.timeline.log('state', state=decision.state, action=decision.action, reason=decision.reason,
                              previous=previous.state if previous else None)
        result = self.executor.act(decision.action, now)
        if result.get('sent') and result['sent'] != 'X':
            # The camera moved: restart the dog-motion window and in-box pixel baseline.
            self.behavior.motion.samples.clear()
            self.prev_gray = None
            self.pre_move_gray = self.last_gray
            self.timeline.log('drive', action=decision.action, pulse=result['sent'], reason=decision.reason)
        elif result.get('sent') == 'X' and (previous is None or previous.action != decision.action):
            self.timeline.log('drive', action=decision.action, pulse='X', reason=decision.reason)
        with self.lock:
            self.decision = decision
        return decision

    def _annotate(self, boxed, found, decision, frame):
        drive = self.executor.last or {}
        sent = drive.get('sent') or ('disarmed' if drive.get('why') == 'disarmed' else '-')
        lines = [f'{decision.state.upper()}  intent: {decision.action}  drive: {sent}', decision.reason]
        if found:
            m = self.behavior.motion
            lines.append(f"dog {found['confidence']:.2f}  {'MOVING' if m.moving else 'still'} "
                         f'{m.still_for(time.monotonic()):.0f}s  speed {m.speed:.2f}')
        if self.describer.latest:
            lines.append(f"{self.describer.latest['state']}: {self.describer.latest['activity']}"[:90])
        cv2.rectangle(boxed, (0, 0), (boxed.shape[1], 18 * len(lines) + 6), (15, 18, 22), -1)
        for i, text in enumerate(lines):
            cv2.putText(boxed, text, (6, 16 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (225, 240, 230), 1, cv2.LINE_AA)
        ok, jpeg = cv2.imencode('.jpg', boxed, [cv2.IMWRITE_JPEG_QUALITY, 85])
        with self.lock:
            self.annotated = jpeg.tobytes()

    def status(self):
        now = time.monotonic()
        m = self.behavior.motion
        with self.lock:
            d = self.decision
        latest = self.describer.latest
        return {
            'drive': self.executor.last,
            'state': d.state if d else None, 'intent': d.action if d else None, 'reason': d.reason if d else None,
            'camera_ok': self.camera_ok, 'dog_visible': self.visible,
            'moving': m.moving if self.visible else None,
            'still_for_s': round(m.still_for(now)) if self.visible else None,
            'activity': latest['activity'] if latest else None,
            'dog_state': latest['state'] if latest else None,
            'activity_age_s': round(now - latest['at']) if latest else None,
            'describe_error': self.describer.error,
            'sleep_after_s': self.behavior.settings.sleep_after_s,
            'frames_processed': self.processed,
            'events': list(self.timeline.recent)[-30:],
        }

    def run(self):
        while True:
            started = time.monotonic()
            try:
                self.step()
            except Exception as error:
                self.timeline.log('watcher_error', error=f'{type(error).__name__}: {error}')
                time.sleep(1)
            time.sleep(max(0.0, FRAME_PERIOD_S - (time.monotonic() - started)))


def serve(watcher, port):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, content_type='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/status':
                return self.reply(200, watcher.status())
            if path == '/frame.jpg':
                with watcher.lock:
                    jpeg = watcher.annotated
                return self.reply(200, jpeg, 'image/jpeg') if jpeg else self.reply(503, {'error': 'no frame yet'})
            self.reply(404, {'error': 'not found'})

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--sleep-after', type=float, default=Settings.sleep_after_s,
                        help='seconds still (plus model says sleeping) before parking')
    args = parser.parse_args()
    if not WEIGHTS.is_file():
        WEIGHTS.parent.mkdir(exist_ok=True)
        os.chdir(WEIGHTS.parent)  # ultralytics downloads known weights into the working directory
    ensure_ollama()
    link = XiaoLink(args.host).start()
    watcher = Watcher(link, Settings(sleep_after_s=args.sleep_after))
    serve(watcher, args.port)
    watcher.timeline.log('watcher_start', host=args.host, sleep_after_s=args.sleep_after)
    watcher.run()


if __name__ == '__main__':
    main()
