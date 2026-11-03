# Mini-T handheld remote → rover

The stock CyberBrick pairing can't reach the rover: its receiver runs the Mini-T signed wireless
service instead of the stock RC program, and only obeys ESP-NOW drive frames (W/A/S/D/X) from the
Mini-T USB transmitter's MAC (20:6E:F1:45:0F:1C). So the remote (20:6E:F1:4F:6E:80) runs a small
program that takes that MAC and sends the same frames as the Mac. Nothing on the receiver changes.

The top-left switch picks the mode (middle = drive; the side that reads 0 = steering calibration,
the other = speed calibration):

| Control | Drive (middle) | Steering calibration | Speed calibration |
| --- | --- | --- | --- |
| Left stick up/down | Forward / reverse | — | Test drive |
| Left stick left/right | — | Centre −1° / +1° (wheels move at once) | — |
| Right stick left/right | Steer (servo only on its own) | Full lock (shows the range) | Steer |
| Top-right lever (flick; hold repeats) | Speed −50 / +50 (saved at once) | Range −5° / +5° | Speed −50 / +50 |
| Top-left button | STOP | Save | Save |

Edits apply on the rover immediately (rover app 0.5 calibration codes: `D A D A` back-to-back plus
two keys, which driving never produces; code keys never drive). They're saved on the receiver
(`rover_cal.json`, survives power-off) with the button or when the switch goes back to the middle.
The rover's version string carries the values (`c130 t45 s900`, `*` = unsaved); the drive server
copies saved ones into `drive/calibration.json` when it starts. Limits: centre 45–135°, range
0–90° and centre ± range within the servo's 0–180°, speed 200–1600.

Measured 2026-11-03: button on pin 6 (0 = pressed), switch 0 / ~1800 / 4076, lever up = high,
left stick up = high, right stick left = low. Left stick left/right direction is assumed (not in
the recording): if centre goes the wrong way, flip `CENTRE_SIGN`.

LED: green drive, purple steering, yellow speed, blue driving, red no answer, white flash saved.
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
