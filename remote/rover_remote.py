"""Mini-T handheld remote -> rover. Runs on the remote (MicroPython, ESP32-C3).

The rover receiver only obeys ESP-NOW drive frames from the Mini-T USB transmitter's MAC and
only knows W/A/S/D/X, so the remote takes that MAC and sends the same frames the Mac does.
Throttle (W/S) and steering (A/D) are both sent every cycle while held; the rover app combines a
throttle key and a steer key that arrive close together (the receiver stops 500 ms after the last
key); letting go sends X until the receiver ACKs. Idle remote sends nothing, so it doesn't fight
the Mac.

Top-left switch picks the mode (middle: drive, one side: steering calibration, other: speed):

  control              drive               steering calibration        speed calibration
  left stick up/down   forward/reverse     -                           test drive
  left stick l/r       -                   centre -1/+1 deg            -
  right stick l/r      steer               full lock (shows range)     steer
  top-right lever      speed -50/+50       range -5/+5 deg             speed -50/+50
  top-left button      STOP                save                        save

Edits go to the rover as calibration codes (rover app 0.5) and apply at once; they're saved on
the rover with the button, when the switch goes back to drive, and right away in drive mode.
LED: green drive, purple steering, yellow speed, blue driving, red no answer, white flash saved.

No auto-sleep: the stock deep sleep wakes on a pin going high, which the mid-voltage sticks never
do, so the remote looked disconnected until restarted (use the power switch). Errors in the loop
are logged and the radio re-initialised; the program never exits.
"""
import time

TRANSMITTER_MAC = '206ef1450f1c'   # the receiver only accepts this sender
RECEIVER_MAC = '206ef14d8d04'
CHANNEL = 1
DRIVE = b'cbdrive1:'

# Stock channel order (rc_config "sender"): value = 4095 - ADC on pins 0..5. Measured 2026-11-03:
# switch middle ~1800, ends 0 / 4076; lever up = high; left stick up = high; right stick left = low.
PIN_SWITCH, PIN_LX, PIN_LY, PIN_LEVER, PIN_RX, PIN_RY = range(6)
PIN_BUTTON = 6                    # top-left button, reads 0 while pressed
MID = {PIN_LX: 1990, PIN_LY: 2048, PIN_LEVER: 2032, PIN_RX: 2007}
THROTTLE_SIGN = 1     # +1: left stick up reads above mid
STEER_SIGN = 1        # +1: right stick right reads above mid
CENTRE_SIGN = 1       # +1: left stick right reads above mid (assumed like the right stick)
LEVER_SIGN = 1        # +1: lever up reads above mid
THRESHOLD = 600       # of ~±2000 travel; well past the stock 240 deadzone
SWITCH_LOW, SWITCH_HIGH = 600, 3400
DRIVE_MODE, STEER_MODE, SPEED_MODE = 'drive', 'steer', 'speed'
REPEAT_MS = 50
STEP_REPEAT_MS = (600, 300)       # hold a flick: first repeat after 600 ms, then every 300 ms
REINIT_AFTER_FAILS = 40           # consecutive undelivered frames (~2 s of driving) -> restart the radio
CODE_PREFIX = 'DADA'              # rover app 0.5 calibration code prefix
CODES = {('center', 1): 'WW', ('center', -1): 'WS', ('travel', 1): 'AW', ('travel', -1): 'AS',
         ('speed', 1): 'DW', ('speed', -1): 'DS', 'save': 'SS'}
LED = {DRIVE_MODE: (0, 40, 0), STEER_MODE: (30, 0, 40), SPEED_MODE: (40, 30, 0)}


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


def mode_for(switch):
    if switch < SWITCH_LOW:
        return STEER_MODE
    if switch > SWITCH_HIGH:
        return SPEED_MODE
    return DRIVE_MODE


def switch_centred(value):
    return mode_for(value) == DRIVE_MODE


class Flick:
    """A held deflection -> one step now, then repeats while held."""
    def __init__(self):
        self.held, self.next_at = 0, 0

    def step(self, direction, now):
        if direction == 0:
            self.held = 0
            return 0
        if direction != self.held:
            self.held, self.next_at = direction, now + STEP_REPEAT_MS[0]
            return direction
        if now >= self.next_at:
            self.next_at = now + STEP_REPEAT_MS[1]
            return direction
        return 0


def edits_for(mode, lx, lever, centre_flick, lever_flick, now):
    """Calibration edits ((what, +/-1), ...) from the left stick and the lever in this mode."""
    edits = []
    step = lever_flick.step(deflection(lever, PIN_LEVER, LEVER_SIGN), now)
    if step:
        edits.append(('travel' if mode == STEER_MODE else 'speed', step))
    if mode == STEER_MODE:
        step = centre_flick.step(deflection(lx, PIN_LX, CENTRE_SIGN), now)
        if step:
            edits.append(('center', step))
    else:
        centre_flick.step(0, now)
    return edits


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

    def send_code(edit):
        return all([send(k) for k in CODE_PREFIX + CODES[edit]])

    def save():
        ok = send_code('save') and stop_ack()
        led((60, 60, 60) if ok else (40, 0, 0))
        time.sleep_ms(250)
        return ok

    radio_up()
    print('ROVER REMOTE READY')
    mode = mode_for(read(PIN_SWITCH))
    led(LED[mode] if stop_ack() else (40, 0, 0))
    driving, dirty, fails, stopped = '', False, 0, False
    centre_flick, lever_flick = Flick(), Flick()
    while True:
        try:
            now = time.ticks_ms()
            new_mode = mode_for(read(PIN_SWITCH))
            if new_mode != mode:
                stop_ack()
                if dirty:
                    save()
                    dirty = False
                mode, driving = new_mode, ''
                led(LED[mode])
            pressed = button.value() == 0
            if pressed and not stopped:
                stopped = True
                if mode == DRIVE_MODE:
                    stop_ack()
                    led((40, 0, 0))
                else:
                    save()
                    dirty = False
                    led(LED[mode])
                driving = ''
            elif not pressed and stopped:
                stopped = False
                led(LED[mode])
            for edit in edits_for(mode, read(PIN_LX), read(PIN_LEVER), centre_flick, lever_flick, now):
                send_code(edit)
                if mode == DRIVE_MODE:
                    save()
                    led(LED[mode])
                else:
                    dirty = True
            keys = '' if stopped else keys_for(read(PIN_LY), read(PIN_RX))
            if mode == STEER_MODE:
                keys = keys.replace('W', '').replace('S', '')      # steering only, no driving
            if keys:
                ok = all([send(k) for k in keys])
                fails = 0 if ok else fails + 1
                led((0, 0, 60) if ok else (40, 0, 0))
                driving = keys
            elif driving:
                ok = stop_ack()
                fails = 0 if ok else fails + 1
                led(LED[mode] if ok else (40, 0, 0))
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
