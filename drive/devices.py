"""This rover's boards: USB serial numbers and radio MACs, kept out of git in drive/devices.json
(copy devices.example.json). The receiver only obeys drive frames from the transmitter's MAC, which
is its USB serial number; the remote borrows that MAC."""
import json
from pathlib import Path

FILE = Path(__file__).resolve().parent / 'devices.json'
EXAMPLE = FILE.with_name('devices.example.json')


def load():
    data = json.loads((FILE if FILE.exists() else EXAMPLE).read_text())
    return {k: data[k].upper() for k in ('transmitter_serial', 'remote_serial', 'receiver_mac')}


def hexmac(mac):
    """'20:6E:F1:...' -> '206ef1...' (what MicroPython's bytes.fromhex wants)."""
    return mac.replace(':', '').lower()
