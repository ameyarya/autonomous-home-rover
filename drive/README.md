# Drive path

Mac → USB → Mini-T CyberBrick transmitter → ESP-NOW → rover receiver → motors + steering servo.
No CyberBrick firmware is flashed: the receiver runs the Mini-T project's signed wireless app
service (500 ms hold-to-drive watchdog, X stop with ACK, app rollback), and the rover's drive
behaviour is an app installed into it over the air.

| File | What |
| --- | --- |
| `rover_drive_app.py` | Receiver app (class `TankApp`, as the service expects). 0.8 (2026-11-03): live calibration codes (centre ±1°, range ±1°, speed ±10, save, undo; `X X X X` + three A/D keys: never drive, even if a code is missed, and the steering stays centred; 0.6's `D A D A` prefix twitched the servo, and 0.6 codes sent to 0.7 reversed the motors briefly, so the drive server refuses live changes unless the rover runs its own app version) sent by the drive server's `POST /calibration/live`; saved on the receiver in `rover_cal.json` and shown in the version string. 0.4: W forward, S reverse, A/D steer only, X stop + centre; a throttle key and a steer key under 400 ms apart combine, so the sender repeats both to drive and steer at once (the Mac sends W with A/D for its forward arcs, so autonomy is unchanged). Steering re-centres 500 ms after the last command. Calibration constants at the top (rewritten from `calibration.json` on install). |
| `calibration.json` + `calibration.py` | Drive calibration: steering servo port, speed, motor directions, steering direction, centre and range. Validated, rendered into the app (version `rover-drive-0.4+c<hash>` shows which values are on the rover) and installed with the signed update. Calibrate page (http://rover.local:8010/calibrate): centre, range and speed sliders change the rover live (no install), Save on rover / Undo; hold-to-steer and test buttons; wiring (motor directions, steering swap, servo port) is a signed install (`POST /calibration` with `install`). Saved live values are copied into `calibration.json` on save and when the drive server starts. |
| `drive_server.py` | Owns the transmitter USB port; HTTP on 127.0.0.1:8769. `GET/POST /calibration` reads, saves and installs the calibration. Pulses only while armed; the arm lease (3 s) must be renewed by the dashboard, otherwise it disarms and stops. STOP aborts a running pulse or hold. Keys: W/S, A/D (forward arcs, used by autonomy), L/R (steer only), Z/C (reverse while steering). Pulse caps: W/A/D 600 ms, S/Z/C 400 ms, L/R 2 s. Manual hold-to-drive (`/hold`): drives while the dashboard renews every 150 ms, stops with ACK 0.35 s after the last renewal; autonomous pulses are refused (409) for 2 s after manual driving. |
| `test_drive_server.py` | Lease expiry, refusal while disarmed, caps, STOP aborting a pulse; hold drives while renewed then stops, STOP ends a hold, autonomy waits after manual driving (fake transmitter). |
| `rover_drive.py` | Mac tool (uses the Mini-T remote on USB, or failing that the USB transmitter): `--status`, `--update APP`, `--rollback`, `--pulse KEY MS` (bounded; repeats every 150 ms, always ends with an ACKed stop; max 2 s). |

Transmitter LED: solid green once the receiver ACKs a stop at connect, red if it doesn't (the stock
breathing purple means our link code isn't running).

Run: `python3 mac/dashboard.py` (http://rover.local:8010/), then start the drive server and dog watcher from its header (or run
`python3 drive/drive_server.py` and `python3 mac/dog_watcher.py` yourself), then Arm (STOP / Esc / Space
any time; closing the page disarms). While armed, hold WASD/arrows to drive manually.

Handheld: the Mini-T remote can drive too (same frames, same sender MAC), and can stand in for the USB transmitter here (`find_port` takes either; the board code sets the transmitter's MAC): [remote/README.md](../remote/README.md).

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
