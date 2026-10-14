# Drive path

Mac → USB → Mini-T CyberBrick transmitter → ESP-NOW → rover receiver → motors + steering servo.
No CyberBrick firmware is flashed: the receiver runs the Mini-T project's signed wireless app
service (500 ms hold-to-drive watchdog, X stop with ACK, app rollback), and the rover's drive
behaviour is an app installed into it over the air.

| File | What |
| --- | --- |
| `rover_drive_app.py` | Receiver app (class `TankApp`, as the service expects). W/A/D forward with steering centred/left/right, S reverse, X stop + centre. Steering re-centres 500 ms after the last command. Calibration constants at the top. |
| `rover_drive.py` | Mac tool: `--status`, `--update APP`, `--rollback`, `--pulse KEY MS` (bounded; repeats every 150 ms, always ends with an ACKed stop; max 2 s). |

Hardware IDs: transmitter 20:6E:F1:45:0F:1C, receiver 20:6E:F1:4D:8D:04. The signing key stays in
the Mini-T folder (`.tank_update.key`; override with `ROVER_UPDATE_KEY`); it is never copied here.

Status 2026-10-14: radio link and key verified (`truck-steering-1.2|0`), `rover-drive-0.1` installed
and confirmed. Not yet moved: calibrate motor directions, steering port/centre/travel, then time
stops on USB unplug and link loss, with the wheels off the ground.
