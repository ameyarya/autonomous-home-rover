# Mini-T handheld remote → rover

The stock CyberBrick pairing can't reach the rover: its receiver runs the Mini-T signed wireless
service instead of the stock RC program, and only obeys ESP-NOW drive frames (W/A/S/D/X) from the
Mini-T USB transmitter's MAC. So the remote runs a small
program that takes that MAC and sends the same frames as the Mac. Nothing on the receiver changes.

| Control | Rover |
| --- | --- |
| Left stick up/down | Forward / reverse |
| Right stick left/right | Steer (servo only on its own; drive and steer together with both sticks) |
| Top-left button | STOP (nothing drives while held) |

Calibration (centre, range, speed) is on the Mac's Calibrate page, live: http://rover.local:8010/calibrate.
A remote calibration mode (switch picks drive/steering/speed, lever and button edit) was tried on
2026-11-03 and removed the same day: hard to remember, and the left stick's direction was a guess.

Measured 2026-11-03: button on pin 6 (0 = pressed), left stick up = high, right stick left = low.

LED: green linked, blue driving, red no answer / STOP held.
The idle remote sends nothing, so it doesn't fight the Mac; while both are on, whichever sent last
wins. No auto-sleep (the stock deep sleep never woke from the sticks, so the remote seemed to
disconnect); switch it off when done. A loop error or ~2 s of undelivered frames restarts the
radio instead of stopping the program.

**Back to the Mini-T tank:** hold the top-left button while powering on (one boot), or `--restore`
(permanent).

```sh
python3 remote/install_remote.py --watch      # live stick values + the key they'd send (sends nothing)
python3 remote/install_remote.py --link       # one stop to the receiver: ACK or not
python3 remote/install_remote.py --install    # backs up stock boot.py (remote/backup/, /boot_stock.py)
python3 remote/install_remote.py --restore
```

Stick reading: value = 4095 − ADC on pins 0–5 (stock order: L shoulder switch, L-x, L-y, R shoulder
stick, R-x, R-y), checked against the stock `rc_module` readings. The stock `rc_module` isn't used:
once started it holds the radio (ESP-NOW sends time out, changing the MAC hangs).
