"""Mini-T handheld remote -> rover. Runs on the remote (MicroPython, ESP32-C3). Driving only.

The rover receiver only obeys ESP-NOW drive frames from the Mini-T USB transmitter's MAC and
only knows W/A/S/D/X, so the remote takes that MAC and sends the same frames the Mac does:

  left stick up/down     forward / reverse (W / S)
  right stick left/right steer (A / D; servo only on its own)
  top-left button        STOP (held: nothing drives)

Throttle and steering are both sent every cycle while held; the rover app combines a throttle
key and a steer key that arrive close together (the receiver stops 500 ms after the last key);
letting go sends X until the receiver ACKs. Idle remote sends nothing, so it doesn't fight the Mac.
Calibration is on the Mac's Calibrate page (live; the remote-mode version was hard to use).

LED: green linked, blue driving, red no answer / STOP held.
No auto-sleep: the stock deep sleep wakes on a pin going high, which the mid-voltage sticks never
do, so the remote looked disconnected until restarted (use the power switch). Errors in the loop
are logged and the radio re-initialised; the program never exits.
"""
import time

# Filled in from drive/devices.json (gitignored) by install_remote.py when it copies this file.
TRANSMITTER_MAC = 'aabbcc000001'   # the receiver only accepts this sender
RECEIVER_MAC = 'aabbcc000003'
CHANNEL = 1
DRIVE = b'cbdrive1:'

# Stock channel order (rc_config "sender"): value = 4095 - ADC on pins 0..5. Measured 2026-11-03:
# switch middle ~1800, ends 0 / 4076; lever up = high; left stick up = high; right stick left = low.
PIN_SWITCH, PIN_LX, PIN_LY, PIN_LEVER, PIN_RX, PIN_RY = range(6)   # switch/LX/lever/RY unused
PIN_BUTTON = 6                    # top-left button, reads 0 while pressed
MID = {PIN_LY: 2048, PIN_RX: 2007}
THROTTLE_SIGN = 1     # +1: left stick up reads above mid
STEER_SIGN = 1        # +1: right stick right reads above mid
THRESHOLD = 600       # of ~±2000 travel; well past the stock 240 deadzone
REPEAT_MS = 50
REINIT_AFTER_FAILS = 40           # consecutive undelivered frames (~2 s of driving) -> restart the radio


def deflection(value, pin, sign):
    d = sign * (value - MID[pin])
    return 1 if d >= THRESHOLD else -1 if d <= -THRESHOLD else 0


def keys_for(ly, rx):
    """Stick readings (stock scale) -> keys to send: throttle ('W'/'S') and/or steer ('A'/'D')."""
    keys = ''
    throttle = deflection(ly, PIN_LY, THROTTLE_SIGN)
    if throttle > 0:
        keys += 'W'
    elif throttle < 0:
        keys += 'S'
    steer = deflection(rx, PIN_RX, STEER_SIGN)
    if steer < 0:
        keys += 'A'
    elif steer > 0:
        keys += 'D'
    return keys


def switch_centred(value):
    """Kept for install_remote --watch."""
    return 600 <= value <= 3400


def main():
    import machine
    import network
    import espnow

    adcs = [machine.ADC(machine.Pin(p), atten=machine.ADC.ATTN_11DB) for p in range(6)]
    button = machine.Pin(PIN_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)

    def read(pin):
        return 4095 - adcs[pin].read()

    def led(rgb):
        try:
            import neopixel
            n = neopixel.NeoPixel(machine.Pin(8, machine.Pin.OUT), 1)
            n[0] = rgb
            n.write()
        except Exception:
            pass

    w = network.WLAN(network.STA_IF)
    e = espnow.ESPNow()
    peer = bytes.fromhex(RECEIVER_MAC)

    def radio_up():
        e.active(False)
        w.active(False)
        time.sleep_ms(100)
        w.active(True)
        w.config(mac=bytes.fromhex(TRANSMITTER_MAC))
        w.config(channel=CHANNEL)
        e.active(True)
        try:
            e.add_peer(peer)
        except OSError:
            pass

    def send(key):
        try:
            return e.send(peer, DRIVE + key.encode())
        except OSError:
            return False

    def stop_ack():
        for _ in range(5):
            send('X')
            try:
                host, msg = e.recv(120)
                if host == peer and msg == DRIVE + b'ACK':
                    return True
            except OSError:
                pass
        return False

    radio_up()
    print('ROVER REMOTE READY')
    green, blue, red = (0, 40, 0), (0, 0, 60), (40, 0, 0)
    led(green if stop_ack() else red)
    driving, fails, stopped = '', 0, False
    while True:
        try:
            if button.value() == 0:
                if not stopped:
                    stopped = True
                    stop_ack()
                    led(red)
                driving = ''
            else:
                if stopped:
                    stopped = False
                    led(green)
                keys = keys_for(read(PIN_LY), read(PIN_RX))
                if keys:
                    ok = all([send(k) for k in keys])
                    fails = 0 if ok else fails + 1
                    led(blue if ok else red)
                    driving = keys
                elif driving:
                    ok = stop_ack()
                    fails = 0 if ok else fails + 1
                    led(green if ok else red)
                    driving = ''
            if fails >= REINIT_AFTER_FAILS:
                print('radio restart')
                radio_up()
                fails = 0
        except KeyboardInterrupt:
            raise
        except Exception as error:
            print('remote loop error:', error)
            led((40, 0, 0))
            try:
                radio_up()
            except Exception:
                pass
        time.sleep_ms(REPEAT_MS)
