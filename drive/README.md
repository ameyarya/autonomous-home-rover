# Drive path

Mac → USB → Mini-T CyberBrick transmitter → ESP-NOW → rover receiver → motors + steering servo.
No CyberBrick firmware is flashed: the receiver runs the Mini-T project's signed wireless app
service (500 ms hold-to-drive watchdog, X stop with ACK, app rollback), and the rover's drive
behaviour is an app installed into it over the air.

| File | What |
| --- | --- |
| `rover_drive_app.py` | Receiver app (class `TankApp`, as the service expects). W/A/D forward with steering centred/left/right, S reverse, X stop + centre. Steering re-centres 500 ms after the last command. Calibration constants at the top (rewritten from `calibration.json` on install). |
| `calibration.json` + `calibration.py` | Drive calibration: steering servo port, speed, motor directions, steering direction, centre and range, plus a steering-only bench test (A/D turn the servo without driving; version ends `-steertest`; never leave on: the rover can't turn). Validated, rendered into the app (version `rover-drive-0.3+c<hash>` shows which values are on the rover) and installed with the signed update. Edit from the dashboard's Calibrate page (http://rover.local:8010/calibrate): test pulses, then Install on rover (~10–30 s, disarms first). The receiver only takes W/A/S/D/X, so calibration can't be changed live. |
| `drive_server.py` | Owns the transmitter USB port; HTTP on 127.0.0.1:8769. `GET/POST /calibration` reads, saves and installs the calibration. Pulses only while armed; the arm lease (3 s) must be renewed by the dashboard, otherwise it disarms and stops. STOP aborts a running pulse or hold. Pulse caps: W/A/D 600 ms, S 400 ms. Manual hold-to-drive (`/hold`): drives while the dashboard renews every 150 ms, stops with ACK 0.35 s after the last renewal; autonomous pulses are refused (409) for 2 s after manual driving. |
| `test_drive_server.py` | Lease expiry, refusal while disarmed, caps, STOP aborting a pulse; hold drives while renewed then stops, STOP ends a hold, autonomy waits after manual driving (fake transmitter). |
| `rover_drive.py` | Mac tool: `--status`, `--update APP`, `--rollback`, `--pulse KEY MS` (bounded; repeats every 150 ms, always ends with an ACKed stop; max 2 s). |

Transmitter LED: solid green once the receiver ACKs a stop at connect, red if it doesn't (the stock
breathing purple means our link code isn't running).

Run: `python3 mac/dashboard.py` (http://rover.local:8010/), then start the drive server and dog watcher from its header (or run
`python3 drive/drive_server.py` and `python3 mac/dog_watcher.py` yourself), then Arm (STOP / Esc / Space
any time; closing the page disarms). While armed, hold WASD/arrows to drive manually.

Hardware IDs: transmitter 20:6E:F1:45:0F:1C, receiver 20:6E:F1:4D:8D:04. The signing key stays in
the Mini-T folder (`.tank_update.key`; override with `ROVER_UPDATE_KEY`); it is never copied here.

Assembled 2026-10-29: steering servo on port S1 (set on the Calibrate page).

Bench results 2026-10-14 (bare motors and servo on the receiver, servo on port S2):

- Radio link and signing key verified (`truck-steering-1.2|0`); `rover-drive-0.2` installed and confirmed.
- W 300 ms: both motors ran, stop ACKed 324 ms after start. A/D 600 ms: stop ACKed at 624–625 ms.
- Mac crash: holding W, the Mac process was SIGKILLed after 1 s with no stop sent; the receiver
  reported stopped afterwards (its 500 ms watchdog). Visual stop timing still to confirm.
- Steering servo: the first servo on S2 was a continuous-rotation (360°) one (±20° command gave
  ~270°), unusable for steering. Replaced with the Mini-T's disc-launcher servo, a positional 180°
  CyberBrick servo: short and long A pulses now give the same small deflection, D the other way,
  and it re-centres. Ready for the linkage (cut its horn per OpenFrame guide step 3.2).
- Wireless: with the receiver off the Mac's USB and on its own power, the signed status reply and a
  W 300 ms pulse worked (stop ACKed at 324 ms), same as on USB.
- Not checkable on bare motors: which motor direction is forward and which way A/D steer; calibrate
  `MOTOR_SIGNS`, `STEER_CENTER`, `STEER_TRAVEL` once the chassis is assembled. Still to test: USB
  unplug and transmitter power loss mid-drive.
