# mac

Mac side of the rover camera pipeline: receives the XIAO's video and telemetry, measures frame
age, refuses stale frames, and feeds the detector ported from
[ai-driven-mini-t](https://github.com/ameyarya/ai-driven-mini-t). Nothing here sends drive commands.

| File | What |
| --- | --- |
| `xiao_link.py` | `XiaoLink`: background MJPEG + WebSocket readers, XIAO→Mac clock mapping, `fresh_frame()` stale gate, `save()` in Mini-T frame naming. `python3 xiao_link.py` prints live link stats. |
| `dashboard.py` + `dashboard.html` | http://rover.local:8010/ (any device on the home Wi-Fi, advertised over mDNS) or http://localhost:8010/, the rover dashboard (redesigned 2026-11-03, dog watcher dropped from the page): big live view with an amber frame while armed; drive chip to start/stop the drive server (log `mac/drive-server.log`); Arm/STOP and hold-to-drive with throttle and steering decoupled (W/S or ↑/↓ drive, A/D or ←/→ only turn the wheels, both together drive and steer, including reverse); camera health with an alert whenever the XIAO restarts (reset reason, e.g. a magnetic-cable power drop); camera settings; link to the live Calibrate page. The dog watcher still runs on its own (`python3 mac/dog_watcher.py`, status on :8768) but isn't shown. Other devices log in with the password in `mac/.dashboard-password` (created on first run, gitignored; edit it to change); this Mac needs no login; STOP works without login. `--bind 127.0.0.1` keeps it to this Mac. Port 80 (a bare http://rover.local/) is taken by another local app, hence :8010. |
| `rover_detector.py` | Mini-T detector (YOLO-World / bullseye matcher), ported with only paths and port (8767) changed. |
| `dog_watcher.py` | Phase 1 dog companion: YOLO11s finds the dog, motion/stillness, Qwen3-VL describes what he is doing, timeline log, and drives through `drive_client.py` when the dashboard has armed the drive server. Status on :8768. |
| `drive_client.py` | Watcher decisions → bounded pulses via the drive server (forward W 300 ms, back S 250, turn = forward arc A/D 300, scan = alternating arc A/D 500 + straight reverse S 300, park/stop X), then 0.6 s of post-stop frames before the next move. Disarmed: logs what it would send. |
| `test_drive_client.py` | Disarmed records only, observe dwell, hold sends nothing, single stop on park, server down, scan arc/reverse alternation, pulses within the server's limits. |
| `dog_behavior.py` | The companion state machine (parked → following → searching), pure logic. |
| `test_dog_behavior.py` | State machine tests: follow on movement, tail-wag counts, park after sleep, search then park, back off when charged, stale camera stops. |
| `depth_service.py` | Camera-only obstacle sensing: Depth Anything V2 Small (Apple GPU) on the dashboard's live frames. Fits the floor at the bottom of the picture, marks anything nearer than the floor at its height, and turns the free floor in the left/centre/right thirds into cm from floor geometry (`--bottom-cm` = how far ahead the picture's bottom edge meets the floor). Port 8774; started from the dashboard's Obstacles card (runs in the detector Python). Model code + weights in `mac/weights/` (gitignored): `git clone --depth 1 https://github.com/DepthAnything/Depth-Anything-V2 mac/weights/depth-anything-v2` (keep `depth_anything_v2/`) and `depth_anything_v2_vits.pth` from huggingface.co/depth-anything/Depth-Anything-V2-Small. Tests: `cd mac && .vision-venv/bin/python -m unittest test_depth_service`. |
| `capture.py` | Photo capture for rebuilding the apartment later: with **Record** on (camera strip on the dashboard), saves 2 frames/s while the rover moves to `mac/captures/<date>/` (gitignored), each with a JSON sidecar (time, frame age, odometer, heading stick, depth reading). Stops itself under 1 GB free disk. Tests: `cd mac && python3 -m unittest test_capture`. |
| `floor_map.py` | Top-down occupancy map (5 cm cells) from the depth service's free floor per column and the rover's pose (drive-time distance + gyro heading); free, obstacle, unknown; rendered with the trail and a 1 m bar (dashboard Obstacles card, View: map). Tests: `.vision-venv/bin/python -m unittest test_floor_map`. |
| `explore.py` | Camera-only exploration in small measured moves while mapping (forward when the centre is clear, 15° arcs to open space, multi-point turns from gyro reverse and forward arcs when blocked). Waits while a dog, cat or person is in view; a guard stops mid-move if one appears or the centre closes in; stops on bumps, stale sensors, stuck steering. Records photos. Tests: `python3 -m unittest test_explore`. |
| `sim.html` | The Sim page (`/sim`): edit the simulator's apartment over the floor plan (move, resize, rotate, add, delete furniture; 3D view); Save writes `sim/apartment/furniture.json` (local) and rebuilds it. The layout is loaded from this Mac when the page opens, not stored in the repo. |
| `shadow.py`, `shadow.html` | Shadow mode (Shadow switch on the dashboard): while you drive from the dashboard, 8 samples/s go to `mac/shadow/<session>/` (local): camera frame, your key, gyro, odometer, depth sectors, what it sees, and what the explorer would have done. The Shadow report (`/shadow`) shows agreement, a you-vs-explorer table, the riskiest disagreements with frames, and explorer thresholds tuned to your driving (safety floors: forward 60 cm, turns 35 cm); Apply writes `mac/explore_params.json`, which `explore.py` loads. The handheld remote bypasses the Mac, so drive from the dashboard. Tests: `python3 -m unittest test_shadow`. |
| `test_dashboard_ui.py` | Browser tests (Playwright + installed Chrome) of the real page and server against fake camera, drive server and watcher: live view and placeholders, health tiles, restart alert, settings, service buttons, arm/hold/release/Esc driving, steer-only left/right and combined drive+steer, live Calibrate page, phone width, LAN login and STOP without login. Setup once: `python3 -m venv mac/.ui-venv && mac/.ui-venv/bin/pip install playwright`; run: `cd mac && .ui-venv/bin/python -m unittest test_dashboard_ui`. |
| `test_xiao_link.py` | Tests against a fake XIAO: clock mapping, frame age, stale detection, telemetry loss, camera fault, captured-after waits. |

Python 3 standard library only; the detector runs in its own environment (below).

## Run

```sh
python3 mac/dashboard.py                    # or --host <camera-ip> if mDNS is slow; start the rest from its header
python3 mac/xiao_link.py --seconds 10       # link stats only
cd mac && python3 -m unittest test_xiao_link
```

The **Observe** button (Detector check, in the side panel) is optional plumbing (Phase 1 has no vision target): give it a detector prompt and it waits
for a frame captured after the click, saves it to `mac/vision-output/` (gitignored), runs the
detector, and shows the labeled result with the frame's age.

## Dog watcher

```sh
python3 mac/dog_watcher.py                  # or Start in the dashboard header; add --sleep-after 60 to test parking faster
python3 mac/dashboard.py                    # dog pane, timeline and watcher frame at http://localhost:8010/
cd mac && python3 -m unittest test_dog_behavior
```

It re-runs itself under the detector Python and starts Ollama if needed. Needs
`mac/weights/yolo11s.pt` (downloaded on first run, gitignored) and the Ollama model
`qwen3-vl:4b-instruct`.

| Rover state | Does | Leaves when |
| --- | --- | --- |
| parked | holds still, watches | dog moves (box moves ≥0.3 box-heights/s, or pixels inside his box change: wagging, head up) → following |
| following | back up straight if he's too close (>55% of frame height) or his box grows >25%/s; else arc toward him to centre him (±10%); else forward if he's small (<30%) | still ≥ `--sleep-after` (300 s) and the model says sleeping (or 2× that and resting) → parked; out of view 3 s → searching |
| searching | rotate roughly in place toward the side he was last seen: forward arc, straight reverse, repeat (~19° per pair) | dog found → following; 75 s → parked |

The vision model describes him when he appears, when he starts/stops moving, and every 60 s while
visible (at most every 15 s). Its state (`sleeping` / `resting` / `active` / `unclear`) gates parking.

Log: `mac/dog-log/YYYY-MM-DD.jsonl` (events `seen`, `lost`, `moving`, `still`, `activity`,
`state`, `camera_stale`) with snapshots in `mac/dog-log/snapshots/` (gitignored, stays on the Mac).

Car steering: the OpenFrame has front-wheel steering and can't spin in place, and the receiver's
drive service only accepts W/A/S/D/X (changing it is off-limits), so there is no reverse with
steering. Turns are forward arcs; rotating in place is an arc + straight-reverse shuffle, a few
degrees per pair (measure on the chassis and tune the pulse lengths).

Blocked moves: after each pulse the watcher compares the whole frame before and after with the camera's
own frame-to-frame noise. A forward or turning move that barely changed the view counts as blocked: two in
a row → back up straight; five → stop ("stuck") and wait 30 s. A blocked reverse → no reversing for 20 s.
Each one is logged as a `blocked` timeline event with the change and threshold, so the simulator-tuned
factors can be checked on the real camera.

Limits: motion is judged only while the rover is stopped (camera motion would look like dog
motion); stillness thresholds are first guesses to tune on the real dog; no drive path yet.

First run on the real dog (2026-10-14, dim living room, camera at desk height): detected
walking down the hall, close up, and with only his face visible (confidence 0.44–0.63); the model
described him as "walking toward the door" and "walking toward the camera", matching the
snapshots. Fixed after that run: moving/still flickered 4× in a second with in-box pixel change
hovering at the threshold (now hysteresis, 2 s to confirm still), and the model's input image could
pick up the status overlay (now its own copy).

Checked 2026-10-14: ~5 frames/s processed live; YOLO11s ~20 ms/frame on MPS (4 s first call);
test dog photo detected at 0.88; Qwen described it as "standing alert on grass" / active in 8 s
(25 s on a cold model load).

## How frame age is measured

The XIAO stamps every frame (`X-Timestamp-Us`) and telemetry message (`t_us`) with its uptime
clock. For each telemetry message, `mac_receive_time - t_us` is the clock offset plus that message's
network delay; the minimum over the last ~10 s is used as the offset. Frame age is then
`now - (frame_timestamp + offset)`. It can slightly understate the true age (by the smallest
telemetry delay seen, a few ms), never overstate it. A device restart (uptime going backwards)
resets the mapping.

A frame is handed to the planner only if: telemetry is under 1 s old, the XIAO reports `cam_ok`,
the clock is mapped, the frame is under 500 ms old, and (for Observe) it was captured after the
request. Otherwise `fresh_frame()` raises `StaleFrame`.

## Detector environment

`dashboard.py` starts `rover_detector.py` on demand with, in order: `ROVER_DETECTOR_PYTHON`; the
Mini-T `.detector-venv` Python; or, if that venv's links are broken, the Mini-T bundled CPython 3.12
with the venv's site-packages on `PYTHONPATH`. Weights default to the Mini-T
`tools/yolov8s-worldv2.pt` (`ROVER_DETECTOR_WEIGHTS` overrides). The bullseye matcher needs
`mac/vision-output/bullseye-reference.jpg`.

## Measured (2026-10-14, desk, XIAO at VGA q12, RSSI about -31 to -47 dBm)

- 26.5–28 fps received; frame age on arrival p50 55–57 ms, p95 78–79 ms, max 114–131 ms.
- 0 stale health samples over 10 s runs; three simultaneous stream clients each at ~28 fps.
- Observe: fresh frame 48–77 ms old, captured 60–75 ms after the click; detector 280–310 ms
  warm (492 ms first call, 8.5 s cold start); 350–400 ms total.
- XIAO reboot: link flagged stale 0.84 s after the reboot command, recovered in 4.6 s and counted
  the device restart. In tests, stopped frames are flagged within the 0.3 s test limit + 0.15 s.
