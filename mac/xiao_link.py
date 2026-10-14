"""Mac side of the XIAO camera link: MJPEG frames + WebSocket telemetry with frame-age checks.

The XIAO stamps every frame (X-Timestamp-Us) and telemetry message (t_us) with its own
uptime clock. XiaoLink maps that clock onto the Mac's monotonic clock using the telemetry
stream, so every frame gets a measured age. A frame whose age is unknown or above
max_frame_age_s is stale and is never handed to the planner.

Run: python3 mac/xiao_link.py [--host rover-cam.local] [--seconds 10]   (prints link stats)
This module only observes; it sends no drive commands.
"""
import argparse
import base64
import collections
import json
import os
import socket
import statistics
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

DEFAULT_HOST = os.environ.get('ROVER_CAM_HOST', 'rover-cam.local')
MAX_FRAME_AGE_S = 0.5
MAX_TELEMETRY_AGE_S = 1.0
CLOCK_WINDOW = 50  # telemetry samples, about 10 s at 5 Hz


class StaleFrame(RuntimeError):
    """No frame fresh enough to act on."""


@dataclass
class Frame:
    jpeg: bytes
    seq: int
    device_us: int        # capture time on the XIAO uptime clock
    received: float       # Mac monotonic time the frame finished arriving
    captured: float | None  # capture time on the Mac monotonic clock; None until the clock is mapped

    def age(self, now=None):
        if self.captured is None:
            return None
        return (time.monotonic() if now is None else now) - self.captured


class ClockMap:
    """Maps XIAO uptime (microseconds) onto Mac monotonic seconds.

    Each telemetry message gives mac_receive_time - device_time, which is the true offset
    plus that message's network delay. The minimum over a sliding window is the best
    estimate. A device restart (uptime going backwards) clears the window.
    """

    def __init__(self, window=CLOCK_WINDOW):
        self._samples = collections.deque(maxlen=window)
        self._last_device_us = None
        self.restarts = 0

    def observe(self, device_us, mac_time):
        if self._last_device_us is not None and device_us < self._last_device_us:
            self._samples.clear()
            self.restarts += 1
        self._last_device_us = device_us
        self._samples.append(mac_time - device_us / 1e6)

    def to_mac(self, device_us):
        if not self._samples:
            return None
        return device_us / 1e6 + min(self._samples)


def _ws_connect(host, port, path, timeout):
    sock = socket.create_connection((host, port), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f'GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n'
                  f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                  'Sec-WebSocket-Version: 13\r\n\r\n').encode())
    reader = sock.makefile('rb')
    status = reader.readline()
    if b' 101 ' not in status:
        sock.close()
        raise ConnectionError('WebSocket upgrade refused: ' + status.decode(errors='replace').strip())
    while reader.readline() not in (b'\r\n', b''):
        pass
    return sock, reader


def _ws_text_messages(sock, reader):
    """Yield text payloads from an unfragmented server-to-client WebSocket."""
    while True:
        head = reader.read(2)
        if len(head) < 2:
            raise ConnectionError('telemetry socket closed')
        opcode, length = head[0] & 0x0F, head[1] & 0x7F
        if length == 126:
            length = int.from_bytes(reader.read(2), 'big')
        elif length == 127:
            length = int.from_bytes(reader.read(8), 'big')
        mask = reader.read(4) if head[1] & 0x80 else None
        payload = reader.read(length)
        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        if opcode == 0x8:
            raise ConnectionError('telemetry socket closed by device')
        if opcode == 0x9:  # ping -> masked pong
            client_mask = os.urandom(4)
            sock.sendall(bytes([0x8A, 0x80 | len(payload)]) + client_mask +
                         bytes(b ^ client_mask[i % 4] for i, b in enumerate(payload)))
        elif opcode == 0x1:
            yield payload


class XiaoLink:
    def __init__(self, host=DEFAULT_HOST, max_frame_age_s=MAX_FRAME_AGE_S,
                 max_telemetry_age_s=MAX_TELEMETRY_AGE_S, http_port=80, stream_port=81, io_timeout=3.0):
        self.host = host
        self.max_frame_age_s = max_frame_age_s
        self.max_telemetry_age_s = max_telemetry_age_s
        self.http_port = http_port
        self.stream_port = stream_port
        self.io_timeout = io_timeout
        self.clock = ClockMap()
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._threads = []
        self._frame = None            # (jpeg, seq, device_us, received)
        self._telemetry = None
        self._telemetry_received = None
        self._receive_times = collections.deque(maxlen=64)
        self.receive_ages = collections.deque(maxlen=2000)  # frame age on arrival, seconds
        self.frames_received = 0
        self.stream_connected = False
        self.telemetry_connected = False
        self.errors = collections.Counter()
        self.last_error = None

    # ----- lifecycle -----

    def start(self):
        for target, name in ((self._stream_loop, 'xiao-stream'), (self._telemetry_loop, 'xiao-telemetry')):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def stop(self):
        self._stop.set()
        with self._cond:
            self._cond.notify_all()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def _note_error(self, source, error):
        with self._cond:
            self.errors[source] += 1
            self.last_error = f'{source}: {type(error).__name__}: {error}'

    # ----- readers -----

    def _stream_loop(self):
        url = f'http://{self.host}:{self.stream_port}/stream'
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=self.io_timeout) as response:
                    self.stream_connected = True
                    while not self._stop.is_set():
                        self._read_part(response)
            except Exception as error:  # reconnect on any failure
                self._note_error('stream', error)
            self.stream_connected = False
            self._stop.wait(1.0)

    def _read_part(self, response):
        line = response.readline()
        while line and not line.startswith(b'--'):
            line = response.readline()
        if not line:
            raise ConnectionError('stream ended')
        headers = {}
        while True:
            line = response.readline()
            if not line:
                raise ConnectionError('stream ended inside part headers')
            line = line.strip()
            if not line:
                break
            key, _, value = line.decode('latin-1').partition(':')
            headers[key.strip().lower()] = value.strip()
        length = int(headers['content-length'])
        jpeg = response.read(length)
        if len(jpeg) != length:
            raise ConnectionError('short frame')
        self._on_frame(jpeg, int(headers['x-frame-seq']), int(headers['x-timestamp-us']), time.monotonic())

    def _telemetry_loop(self):
        while not self._stop.is_set():
            sock = None
            try:
                sock, reader = _ws_connect(self.host, self.http_port, '/ws', self.io_timeout)
                self.telemetry_connected = True
                for payload in _ws_text_messages(sock, reader):
                    self._on_telemetry(json.loads(payload), time.monotonic())
                    if self._stop.is_set():
                        break
            except Exception as error:
                self._note_error('telemetry', error)
            finally:
                if sock:
                    sock.close()
            self.telemetry_connected = False
            self._stop.wait(1.0)

    def _on_frame(self, jpeg, seq, device_us, received):
        with self._cond:
            self._frame = (jpeg, seq, device_us, received)
            self.frames_received += 1
            self._receive_times.append(received)
            captured = self.clock.to_mac(device_us)
            if captured is not None:
                self.receive_ages.append(max(0.0, received - captured))
            self._cond.notify_all()

    def _on_telemetry(self, message, received):
        with self._cond:
            if isinstance(message.get('t_us'), int):
                self.clock.observe(message['t_us'], received)
            self._telemetry = message
            self._telemetry_received = received
            self._cond.notify_all()

    # ----- queries -----

    def _snapshot(self):
        if self._frame is None:
            return None
        jpeg, seq, device_us, received = self._frame
        captured = self.clock.to_mac(device_us)
        if captured is not None:
            captured = min(captured, received)
        return Frame(jpeg, seq, device_us, received, captured)

    def latest(self):
        """Newest frame regardless of age (for display); use fresh_frame() for decisions."""
        with self._cond:
            return self._snapshot()

    def telemetry(self):
        with self._cond:
            return dict(self._telemetry) if self._telemetry else None

    def _problems(self, frame, now, captured_after=None):
        problems = []
        if self._telemetry_received is None:
            problems.append('no telemetry yet')
        elif now - self._telemetry_received > self.max_telemetry_age_s:
            problems.append(f'telemetry stale ({(now - self._telemetry_received) * 1000:.0f} ms)')
        elif self._telemetry.get('cam_ok') is False:
            problems.append('camera fault reported by XIAO')
        if frame is None:
            problems.append('no frames yet')
        elif frame.captured is None:
            problems.append('frame age unknown (clock not mapped)')
        elif frame.age(now) > self.max_frame_age_s:
            problems.append(f'frame stale ({frame.age(now) * 1000:.0f} ms > {self.max_frame_age_s * 1000:.0f} ms)')
        elif captured_after is not None and frame.captured < captured_after:
            problems.append('waiting for a frame captured after the request')
        return problems

    def health(self):
        with self._cond:
            now = time.monotonic()
            frame = self._snapshot()
            problems = self._problems(frame, now)
            recent = [t for t in self._receive_times if now - t <= 2.0]
            telemetry = self._telemetry or {}
            return {
                'state': 'ok' if not problems else 'stale',
                'problems': problems,
                'host': self.host,
                'stream_connected': self.stream_connected,
                'telemetry_connected': self.telemetry_connected,
                'frame_seq': frame.seq if frame else None,
                'frame_age_ms': round(frame.age(now) * 1000) if frame and frame.captured is not None else None,
                'fps': round((len(recent) - 1) / (recent[-1] - recent[0]), 1) if len(recent) > 2 else 0.0,
                'telemetry_age_ms': round((now - self._telemetry_received) * 1000) if self._telemetry_received else None,
                'max_frame_age_ms': round(self.max_frame_age_s * 1000),
                'device_restarts_seen': self.clock.restarts,
                'errors': dict(self.errors),
                'last_error': self.last_error,
                'telemetry': telemetry,
            }

    def fresh_frame(self, captured_after=None, timeout=2.0):
        """Return a frame that is fresh now (and captured after `captured_after`, a Mac
        monotonic time, if given). Raises StaleFrame if none arrives within `timeout`."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                now = time.monotonic()
                frame = self._snapshot()
                problems = self._problems(frame, now, captured_after)
                if not problems:
                    return frame
                if now >= deadline or self._stop.is_set():
                    raise StaleFrame('; '.join(problems))
                self._cond.wait(min(0.05, deadline - now))

    def save(self, frame, directory):
        """Write the frame as a JPEG plus a .json sidecar, Mini-T naming."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / ('frame-' + time.strftime('%Y%m%d-%H%M%S') + '-' + str(time.time_ns() % 1000000000) + '.jpg')
        path.write_bytes(frame.jpeg)
        meta = {'seq': frame.seq, 'device_us': frame.device_us,
                'age_ms_at_save': round(frame.age() * 1000, 1) if frame.captured is not None else None,
                'source': f'xiao://{self.host}', 'telemetry': self.telemetry()}
        path.with_suffix('.json').write_text(json.dumps(meta, indent=2) + '\n')
        return path


def _percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    with XiaoLink(args.host) as link:
        time.sleep(2)  # connect and map the clock
        link.receive_ages.clear()
        start_frames, start = link.frames_received, time.monotonic()
        stale_samples = samples = 0
        while time.monotonic() - start < args.seconds:
            samples += 1
            stale_samples += link.health()['state'] != 'ok'
            time.sleep(0.1)
        elapsed = time.monotonic() - start
        health = link.health()
        ages = [a * 1000 for a in link.receive_ages]
        telemetry = health['telemetry']
        try:
            with urllib.request.urlopen(f'http://{args.host}/config', timeout=3) as response:
                config = json.load(response)
        except OSError:
            config = {}
        print(f"host {args.host}  fw {telemetry.get('fw')}  {config.get('framesize')} q{config.get('quality')}  "
              f"rssi {telemetry.get('rssi')} dBm")
        print(f'frames {link.frames_received - start_frames} in {elapsed:.1f} s = '
              f'{(link.frames_received - start_frames) / elapsed:.1f} fps')
        if ages:
            print(f'frame age on arrival ms: p50 {statistics.median(ages):.0f}  p95 {_percentile(ages, .95):.0f}  '
                  f'max {max(ages):.0f}')
        print(f'stale {stale_samples}/{samples} health samples; errors {health["errors"] or "none"}')
        print('state', health['state'], health['problems'] or '')


if __name__ == '__main__':
    main()
