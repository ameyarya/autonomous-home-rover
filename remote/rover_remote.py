"""Mini-T handheld remote -> rover. Runs on the remote (MicroPython, ESP32-C3).

The rover receiver only obeys ESP-NOW drive frames from the Mini-T USB transmitter's MAC and
only knows W/A/S/D/X, so the remote takes that MAC and sends the same frames the Mac does:
left stick up/down = W/S (forward/reverse), right stick left/right = A/D (steering only). Both
are sent together every 100 ms while held, and the rover app combines a throttle key and a steer
key that arrive close together, so you can drive and steer at once (the receiver stops 500 ms
after the last key); letting go sends X until the receiver ACKs. Idle remote sends nothing, so
it doesn't fight the Mac. 5 minutes idle -> deep sleep like the stock remote (move a stick to wake).
"""
import time

TRANSMITTER_MAC = '206ef1450f1c'   # the receiver only accepts this sender
RECEIVER_MAC = '206ef14d8d04'
CHANNEL = 1
DRIVE = b'cbdrive1:'

# Stock channel order (rc_config "sender"): value = 4095 - ADC on pins 0..5.
PIN_SWITCH, PIN_LX, PIN_LY, PIN_RSHOULDER, PIN_RX, PIN_RY = range(6)
MID = {PIN_SWITCH: 2200, PIN_LY: 2127, PIN_RX: 2199}   # rc_config mid_value
THROTTLE_SIGN = 1     # +1 if left stick up reads above mid; flip with --watch
STEER_SIGN = 1        # +1 if right stick right reads above mid
THRESHOLD = 600       # of ~±2000 travel; well past the stock 240 deadzone
SWITCH_DEADZONE = 500
REPEAT_MS = 100
IDLE_SLEEP_S = 5 * 60


def keys_for(ly, rx):
    """Stick readings (stock scale) -> keys to send: throttle ('W'/'S') and/or steer ('A'/'D')."""
    throttle = THROTTLE_SIGN * (ly - MID[PIN_LY])
    steer = STEER_SIGN * (rx - MID[PIN_RX])
    keys = ''
    if throttle >= THRESHOLD:
        keys += 'W'
    elif throttle <= -THRESHOLD:
        keys += 'S'
    if steer <= -THRESHOLD:
        keys += 'A'
    elif steer >= THRESHOLD:
        keys += 'D'
    return keys


def switch_centred(value):
    return abs(value - MID[PIN_SWITCH]) <= SWITCH_DEADZONE


def main():
    import machine
    import network
    import espnow

    adcs = [machine.ADC(machine.Pin(p), atten=machine.ADC.ATTN_11DB) for p in range(6)]

    def read(pin):
        return 4095 - adcs[pin].read()

    def led(r, g, b):
        try:
            import neopixel
            n = neopixel.NeoPixel(machine.Pin(8, machine.Pin.OUT), 1)
            n[0] = (r, g, b)
            n.write()
        except Exception:
            pass

    w = network.WLAN(network.STA_IF)
    w.active(True)
    w.config(mac=bytes.fromhex(TRANSMITTER_MAC))
    w.config(channel=CHANNEL)
    e = espnow.ESPNow()
    e.active(True)
    peer = bytes.fromhex(RECEIVER_MAC)
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

    print('ROVER REMOTE READY')
    led(*((0, 40, 0) if stop_ack() else (40, 0, 0)))
    driving = None
    last_active = time.ticks_ms()
    while True:
        keys = keys_for(read(PIN_LY), read(PIN_RX))
        if keys:
            ok = all([send(k) for k in keys])
            led(*((0, 0, 60) if ok else (40, 0, 0)))
            driving = keys
            last_active = time.ticks_ms()
        elif driving:
            led(*((0, 40, 0) if stop_ack() else (40, 0, 0)))
            driving = None
        elif time.ticks_diff(time.ticks_ms(), last_active) > IDLE_SLEEP_S * 1000:
            stop_ack()
            import rc_main
            rc_main.sleep_handler()
        time.sleep_ms(REPEAT_MS)
