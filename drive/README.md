# Drive path

Mac → USB → Mini-T CyberBrick transmitter → ESP-NOW → rover receiver → motors + steering servo.
No CyberBrick firmware is flashed: the receiver runs the Mini-T project's signed wireless app
service (500 ms hold-to-drive watchdog, X stop with ACK, app rollback), and the rover's drive
behaviour is an app installed into it over the air.

| File | What |
| --- | --- |
| `rover_drive_app.py` | Receiver app (class `TankApp`, as the service expects). W/A/D forward with steering centred/left/right, S reverse, X stop + centre. Steering re-centres 500 ms after the last command. Calibration constants at the top. |
| `rover_drive.py` | Mac tool: `--status`, `--update APP`, `--rollback`, `--pulse KEY MS` (bounded; repeats every 150 ms, always ends with an ACKed stop; max 2 s). |

Transmitter LED: solid green once the receiver ACKs a stop at connect, red if it doesn't (the stock
breathing purple means our link code isn't running).

Hardware IDs: transmitter 20:6E:F1:45:0F:1C, receiver 20:6E:F1:4D:8D:04. The signing key stays in
the Mini-T folder (`.tank_update.key`; override with `ROVER_UPDATE_KEY`); it is never copied here.

Bench results 2026-10-14 (bare motors and servo on the receiver, servo on port S2):

- Radio link and signing key verified (`truck-steering-1.2|0`); `rover-drive-0.2` installed and confirmed.
- W 300 ms: both motors ran, stop ACKed 324 ms after start. A/D 600 ms: stop ACKed at 624–625 ms.
- Mac crash: holding W, the Mac process was SIGKILLed after 1 s with no stop sent; the receiver
  reported stopped afterwards (its 500 ms watchdog). Visual stop timing still to confirm.
- Not checkable on bare motors: which motor direction is forward and which way A/D steer; calibrate
  `MOTOR_SIGNS`, `STEER_CENTER`, `STEER_TRAVEL` once the chassis is assembled. Still to test: USB
  unplug and transmitter power loss mid-drive.
