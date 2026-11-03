# Mini-T handheld remote → rover

The stock CyberBrick pairing can't reach the rover: its receiver runs the Mini-T signed wireless
service instead of the stock RC program, and only obeys ESP-NOW drive frames (W/A/S/D/X) from the
Mini-T USB transmitter's MAC (20:6E:F1:45:0F:1C). So the remote (20:6E:F1:4F:6E:80) runs a small
program that takes that MAC and sends the same frames as the Mac. Nothing on the receiver changes.

| Stick (Mini-T remote) | Rover |
| --- | --- |
| Left stick up | W: forward, steering centred |
| Right stick left/right (with or without left stick up) | A / D: turn left/right (forward arc; a car can't turn standing still) |
| Left stick down | S: reverse, straight (the receiver has no reverse with steering) |
| Let go | X until the receiver ACKs (it also stops by itself 500 ms after the last key) |

On the Mini-T these sticks drove the left/right tracks (left Y → MOTOR2, right Y → MOTOR1, right
shoulder stick → turret, left shoulder button → fire). Here the rover has one drive channel plus
steering, so it is car-style: left stick throttle, right stick steering. Speed is the calibrated one
(on/off, no proportional throttle). The right Y stick, shoulder stick and buttons do nothing.

LED: green = receiver ACKed a stop, blue = driving, red = no answer from the receiver.
The idle remote sends nothing, so it doesn't fight the Mac; while both are on, whichever sent last
wins. 5 min idle → deep sleep like stock (move a stick to wake).

**Back to the Mini-T tank:** power the remote on with the left shoulder 3-position switch away from
the middle (one boot), or `--restore` (permanent).

```sh
python3 remote/install_remote.py --watch      # live stick values + the key they'd send (sends nothing)
python3 remote/install_remote.py --link       # one stop to the receiver: ACK or not
python3 remote/install_remote.py --install    # backs up stock boot.py (remote/backup/, /boot_stock.py)
python3 remote/install_remote.py --restore
```

Stick reading: value = 4095 − ADC on pins 0–5 (stock order: L shoulder switch, L-x, L-y, R shoulder
stick, R-x, R-y), checked against the stock `rc_module` readings. The stock `rc_module` isn't used:
once started it holds the radio (ESP-NOW sends time out, changing the MAC hangs).

Installed 2026-11-03; left stick forward/back confirmed on the rover. Right stick now turns on its
own (was: only while the left stick was up); its direction (`STEER_SIGN`) still to confirm.
