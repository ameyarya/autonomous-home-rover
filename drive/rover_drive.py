"""Mac -> USB CyberBrick transmitter -> ESP-NOW -> rover receiver.

Reuses the Mini-T transmitter link (its USB transmitter, REPL board code, and the signed
wireless app-update protocol), pointed at the rover's receiver.

  python3 drive/rover_drive.py --status                 # app version + running flag (radio + key check)
  python3 drive/rover_drive.py --update drive/rover_drive_app.py
  python3 drive/rover_drive.py --rollback
  python3 drive/rover_drive.py --pulse W 300            # hold W for 300 ms, then stop (wheels off the ground!)

Safety: every motion is a bounded pulse. Commands are repeated every 150 ms (the receiver
stops on its own 500 ms after the last one) and always end with X, which the receiver ACKs.
"""
import argparse
import hashlib
import hmac
import os
import secrets
import struct
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

TRANSMITTER_SERIAL = '20:6E:F1:45:0F:1C'   # Mini-T USB transmitter
RECEIVER_MAC = '206ef14d8d04'              # rover receiver (service installed by the Mini-T truck experiment)
KEY_PATH = Path(os.environ.get('ROVER_UPDATE_KEY',
                               Path.home() / 'Documents/Projects/ai-driven-mini-t/.tank_update.key'))
PREFIX = b'CBU1'
REPEAT_S = 0.15
MAX_PULSE_MS = 2000

BOARD_CODE = '''import espnow
import network
import time
e = espnow.ESPNow()
e.active(True)
peer = bytes.fromhex("%s")
try:
 e.add_peer(peer)
except OSError:
 pass
def wire(c):
 for attempt in range(3):
  if e.send(peer,b"cbdrive1:"+c.encode()):
   return
  time.sleep_ms(10)
 raise OSError("Receiver unavailable")
def stop_ack():
 for attempt in range(5):
  try:
   wire("X")
   ack = e.recv(150)
   if ack[0] == peer and ack[1] == b"cbdrive1:ACK":
    return True
  except OSError:
   pass
  time.sleep_ms(20)
 return False
if not stop_ack():
 raise OSError("Receiver did not acknowledge stop")
print("WIRELESS HOLD CONTROL READY")
import ubinascii
def ota(raw):
 packet = bytes.fromhex(raw)
 e.send(peer,packet,False)
 start = time.ticks_ms()
 while time.ticks_diff(time.ticks_ms(),start)<280:
  host,msg = e.recv(0)
  if host==peer and msg and msg.startswith(b"CBU1"):
   print("OTA",ubinascii.hexlify(msg).decode())
   return
  time.sleep_ms(2)
 print("OTA NONE")
''' % RECEIVER_MAC


def find_port():
    if os.environ.get('ROVER_TX_PORT'):
        return os.environ['ROVER_TX_PORT']
    for p in serial.tools.list_ports.comports():
        if (p.serial_number or '').upper() == TRANSMITTER_SERIAL:
            return p.device
    raise RuntimeError(f'Transmitter {TRANSMITTER_SERIAL} not found on USB; set ROVER_TX_PORT')


class Transmitter:
    def __init__(self, port=None):
        self.ser = serial.Serial()
        self.ser.port = port or find_port()
        self.ser.baudrate = 115200
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self._init_board()

    def _read_until_prompt(self, timeout):
        response = bytearray()
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            response.extend(self.ser.read(self.ser.in_waiting or 1))
            if response.endswith(b'>>> '):
                break
        return bytes(response)

    def _init_board(self):
        time.sleep(1.5)
        for _ in range(5):                  # interrupt the board's app -> REPL
            self.ser.write(b'\x03')
            time.sleep(0.3)
        self.ser.read(self.ser.in_waiting or 1)
        self.ser.write(('exec(%r)\r\n' % BOARD_CODE).encode())
        out = self._read_until_prompt(8)
        if b'WIRELESS HOLD CONTROL READY' not in out or b'Traceback' in out:
            raise RuntimeError('Transmitter setup failed: ' + out.decode(errors='replace')[-300:])

    def repl(self, line, timeout=0.5):
        self.ser.write(line.encode() + b'\r\n')
        out = self._read_until_prompt(timeout)
        if b'Traceback' in out or not out.endswith(b'>>> '):
            raise RuntimeError(out.decode(errors='replace'))
        return out

    def wire(self, key):
        self.repl('wire(%r)' % key)

    def stop(self):
        out = self.repl('print("ACK" if stop_ack() else "NOACK")', timeout=1.5)
        if b'\r\nACK' not in out and not out.startswith(b'ACK'):
            raise RuntimeError('Receiver did not acknowledge stop')

    def pulse(self, key, ms):
        if key not in 'WASD':
            raise ValueError('key must be W, A, S or D')
        ms = max(0, min(int(ms), MAX_PULSE_MS))
        end = time.monotonic() + ms / 1000
        try:
            while True:
                self.wire(key)
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(REPEAT_S, remaining))
        finally:
            self.stop()

    # ---- signed app protocol (same as the Mini-T wireless_update.py) ----
    def exchange(self, kind, ident, payload=b''):
        secret = KEY_PATH.read_bytes()
        body = PREFIX + kind + ident + payload
        packet = body + hmac.new(secret, body, hashlib.sha256).digest()[:16]
        for _ in range(5):
            out = self.repl('ota(%r)' % packet.hex(), timeout=1.0).decode(errors='replace')
            for line in out.splitlines():
                if not line.startswith('OTA ') or line == 'OTA NONE':
                    continue
                reply = bytes.fromhex(line[4:])
                signed = reply[:-16]
                if len(reply) < 25 or not hmac.compare_digest(reply[-16:], hmac.new(secret, signed, hashlib.sha256).digest()[:16]):
                    continue
                if signed[:4] != PREFIX or signed[5:9] != ident:
                    continue
                kind_back, result = signed[4:5], signed[9:]
                if kind_back == b'E':
                    raise RuntimeError('Receiver: ' + result.decode(errors='replace'))
                if kind_back == (b'A' if kind in (b'B', b'D') else kind):
                    return result
        raise TimeoutError('Receiver did not answer the signed packet')

    def status(self):
        return self.exchange(b'P', secrets.token_bytes(4)).decode()

    def rollback(self):
        return self.exchange(b'R', secrets.token_bytes(4)).decode()

    def update_app(self, source):
        compile(source, 'rover_drive_app.py', 'exec')
        ident = secrets.token_bytes(4)
        try:
            result = self.exchange(b'B', ident, struct.pack('>I', len(source)) + hashlib.sha256(source).digest())
            if struct.unpack('>I', result)[0] != 0:
                raise RuntimeError('Unexpected initial upload offset')
            offset = 0
            while offset < len(source):
                chunk = source[offset:offset + 160]
                result = self.exchange(b'D', ident, struct.pack('>I', offset) + chunk)
                offset += len(chunk)
                if struct.unpack('>I', result)[0] != offset:
                    raise RuntimeError('Receiver upload offset mismatch')
            version = self.exchange(b'F', ident).decode()
            if self.status() != version + '|0':
                raise RuntimeError('Updated app failed its stopped-state check')
            self.exchange(b'G', ident)
            return version
        except Exception:
            try:
                self.exchange(b'Z', ident)
            except Exception:
                pass
            raise

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--status', action='store_true')
    parser.add_argument('--update', type=Path)
    parser.add_argument('--rollback', action='store_true')
    parser.add_argument('--pulse', nargs=2, metavar=('KEY', 'MS'))
    args = parser.parse_args()
    tx = Transmitter()
    try:
        if args.update:
            print('installed', tx.update_app(args.update.read_bytes()))
        if args.rollback:
            print('rolled back to', tx.rollback())
        if args.pulse:
            key, ms = args.pulse[0].upper(), int(args.pulse[1])
            t0 = time.monotonic()
            tx.pulse(key, ms)
            print(f'pulse {key} {ms} ms done, stop ACKed after {(time.monotonic() - t0) * 1000:.0f} ms')
        if args.status or not (args.update or args.rollback or args.pulse):
            print('receiver app|running:', tx.status())
    finally:
        tx.close()


if __name__ == '__main__':
    sys.exit(main())
