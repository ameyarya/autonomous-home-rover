#!/usr/bin/env python3
"""Install the rover program on the Mini-T handheld remote over USB (or undo it).

  --watch [S]   live stick readings and the key they'd send (nothing is sent)
  --install     back up the stock boot.py (here: remote/backup/, on the remote: /boot_stock.py),
                copy rover_remote.py + boot.py, reboot into rover mode
  --link        one stop (X) to the rover receiver; reports whether it ACKed
                (--watch and --link reboot the remote afterwards)
  --restore     put the stock boot.py back (the Mini-T remote again)
"""
import argparse
import hashlib
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

HERE = Path(__file__).resolve().parent
REMOTE_SERIAL = '20:6E:F1:4F:6E:80'   # Mini-T handheld remote
sys.path.insert(0, str(HERE))
import rover_remote  # noqa: E402


def find_port():
    for p in serial.tools.list_ports.comports():
        if (p.serial_number or '').upper() == REMOTE_SERIAL:
            return p.device
    raise SystemExit(f'Remote {REMOTE_SERIAL} not found on USB')


class Board:
    def __init__(self):
        self.ser = serial.Serial()
        self.ser.port = find_port()
        self.ser.baudrate = 115200
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(1.0)
        for _ in range(4):          # interrupt whatever boot.py is running -> REPL
            self.ser.write(b'\x03')
            time.sleep(0.3)
        self.ser.read(self.ser.in_waiting or 1)

    def run(self, line, timeout=3.0):
        self.ser.write(line.encode() + b'\r\n')
        out, end = b'', time.monotonic() + timeout
        while time.monotonic() < end:
            out += self.ser.read(self.ser.in_waiting or 1)
            if out.endswith(b'>>> '):
                break
        text = out.decode(errors='replace')
        if 'Traceback' in text or not out.endswith(b'>>> '):
            raise RuntimeError(text)
        return text.split('\r\n', 1)[1].rsplit('\r\n>>> ', 1)[0] if '\r\n' in text else ''

    def read_file(self, path):
        return bytes.fromhex(self.run(f"import ubinascii;print(ubinascii.hexlify(open({path!r},'rb').read()).decode())", 6).strip())

    def write_file(self, path, data):
        self.run(f"f=open({path!r},'wb')")
        for i in range(0, len(data), 256):
            self.run(f"f.write(bytes.fromhex({data[i:i + 256].hex()!r}))")
        self.run('f.close()')
        got = self.run(f"import hashlib,ubinascii;print(ubinascii.hexlify(hashlib.sha256(open({path!r},'rb').read()).digest()).decode())")
        if got.strip() != hashlib.sha256(data).hexdigest():
            raise RuntimeError(f'{path}: written file does not match')

    def exists(self, path):
        return self.run(f"import os;print({path.lstrip('/')!r} in os.listdir('/'))").strip() == 'True'

    def reset(self):
        self.ser.write(b'import machine;machine.reset()\r\n')
        time.sleep(0.3)
        self.ser.close()


def watch(board, seconds):
    board.run('import machine;A=[machine.ADC(machine.Pin(p),atten=machine.ADC.ATTN_11DB) for p in range(6)]')
    end = time.monotonic() + seconds
    print('switch  L-x  L-y  R-sh  R-x  R-y   key   (Ctrl-C to quit)')
    while time.monotonic() < end:
        v = [int(x) for x in board.run('print(*[4095-a.read() for a in A])').split()]
        key = rover_remote.keys_for(v[rover_remote.PIN_LY], v[rover_remote.PIN_RX]) or '-'
        mode = 'rover' if rover_remote.switch_centred(v[0]) else 'Mini-T'
        print(f'\r{v[0]:5} {v[1]:5} {v[2]:5} {v[3]:5} {v[4]:5} {v[5]:5}   {key:2}   boot:{mode}   ', end='', flush=True)
        time.sleep(0.15)
    print()


def install(board):
    backup = HERE / 'backup' / 'boot_stock.py'
    if not board.exists('/boot_stock.py'):
        stock = board.read_file('/boot.py')
        if b'rover_remote' in stock:
            raise SystemExit('boot.py is already the rover one but /boot_stock.py is missing; not overwriting')
        backup.parent.mkdir(exist_ok=True)
        backup.write_bytes(stock)
        board.write_file('/boot_stock.py', stock)
        print(f'stock boot.py saved to {backup} and /boot_stock.py')
    board.write_file('/rover_remote.py', (HERE / 'rover_remote.py').read_bytes())
    board.write_file('/boot.py', (HERE / 'boot.py').read_bytes())
    print('installed; rebooting into rover mode')
    board.reset()


def restore(board):
    if not board.exists('/boot_stock.py'):
        raise SystemExit('/boot_stock.py not on the remote; nothing to restore')
    board.write_file('/boot.py', board.read_file('/boot_stock.py'))
    print('stock Mini-T boot.py restored; rebooting')
    board.reset()


def link(board):
    code = ("import network,espnow;w=network.WLAN(network.STA_IF);w.active(True);"
            f"w.config(mac=bytes.fromhex({rover_remote.TRANSMITTER_MAC!r}));w.config(channel={rover_remote.CHANNEL});"
            f"e=espnow.ESPNow();e.active(True);p=bytes.fromhex({rover_remote.RECEIVER_MAC!r})")
    board.run(code)
    board.run('p in [x[0] for x in e.get_peers()] or e.add_peer(p)')
    out = board.run("r=[e.send(p,b'cbdrive1:X') for i in range(3)];m=e.recv(300);print(r,m[1])")
    print('radio delivered:', out.strip())
    print('receiver ACKed stop' if 'ACK' in out else 'NO ACK: is the rover receiver on and in range?')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--watch', nargs='?', type=float, const=60)
    g.add_argument('--install', action='store_true')
    g.add_argument('--link', action='store_true')
    g.add_argument('--restore', action='store_true')
    a = ap.parse_args()
    board = Board()
    if a.install:
        install(board)
    elif a.restore:
        restore(board)
    else:
        try:
            watch(board, a.watch) if a.watch else link(board)
        except KeyboardInterrupt:
            print()
        finally:
            board.reset()       # back to whatever boot.py starts (rover mode)


if __name__ == '__main__':
    main()
