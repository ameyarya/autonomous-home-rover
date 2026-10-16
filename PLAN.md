# Home Rover — Project Plan

Repository: https://github.com/ameyarya/autonomous-home-rover

Status: Phase 1 in progress — chassis print slipped and not yet rescheduled (as of 2026-10-13), XIAO Sense firmware fully configured and bench-verified 2026-10-14 (camera, mic, OTA updates, runtime settings, health), powered by an iPhone Air battery pack; next is the housing and mounting. Phase 2 (hardware/software upgrades) is study-only until Phase 1 is done. This is a living plan; update it as hardware is verified and decisions are made.

## Goal

Build an autonomous indoor rover using the CyberBrick OpenFrame Two 4WD RC car as the base, with an ESP32-S3 and camera mounted on top. The long-term goal is autonomous operation at home. The primary mission is home patrol: visit designated locations and capture camera observations. The navigation method, patrol schedule, and reporting behavior remain to be selected.

Base project: https://makerworld.com/en/models/2552360-cyberbrick-openframe-two-4wd-rc-car#profileId-2850510 (corrected 2026-10-14; earlier entries referenced the OpenFrame One, https://makerworld.com/en/models/2243313)

## Confirmed scope

- Use the frame, drivetrain, and driving electronics exactly as specified by the MakerWorld project.
- Add an ESP32-S3 with a camera on top of the frame.
- The user already owns a CyberBrick kit with receiver and transmitter. Keep these as the driving command path.
- Run navigation on the Mac, connected by USB to the CyberBrick transmitter; the transmitter sends drive commands wirelessly to the rover receiver. This is the selected architecture; USB command injection still needs verification.
- Use the added ESP32 only for camera capture and sensor acquisition, with telemetry sent to the Mac over Wi-Fi. It does not run navigation or send drive commands.
- Maintain this repository throughout the project, including this plan, hardware decisions, firmware, wiring, build notes, and validation results.

## Proposed first release

Start with slow autonomous roaming and obstacle avoidance in one controlled indoor room, with manual charging. This is the initial milestone toward home patrol; detailed acceptance targets remain to be set.

The first release should support manual commissioning, camera capture, autonomous start/stop, obstacle stopping, and a safe stopped state on faults. Room-to-room navigation, person following, scheduled patrols, mapping, and automatic docking are later candidates rather than committed first-release features.

## Phases (locked 2026-10-08)

Roadmap framing (2026-10-16): the Mini-T was the proof of concept; Phase 1 replicates what it did on the rover (Mac brain, camera, bounded autonomy, now aimed at the dog); Phase 2 is the full build, going all out: real sensors, onboard compute, ROS 2 / SLAM / Nav2, and the study items below.

**Phase 1 — Mini-T parity on the rover.** Mac brain, Mac transmission, XIAO camera onboard. Print and assemble the chassis, verify manual control, build a printed housing for the XIAO Sense, bench-bring-up the camera stream, and port the Mini-T stack (server/command path, autonomy loop, detector + dashboard, calibration files) to the wheeled chassis and XIAO pipeline. Done when the rover behaves like the Mini-T: manual drive, live camera, bounded autonomous find/align/approach with safety stops.

**Phase 2 — the full build (study only for now).** Hardware, in order: encoders + IMU → single-point ranging → 2D LiDAR → onboard compute (desk Pi first, then onboard) → split power rails. Software: ROS 2 transports, SLAM (default candidate slam_toolbox), Nav2 + docking. Study the options while executing Phase 1; no purchases and no rewrites until Phase 1 is done.

**Phase 2 reference — OOMWOO (reviewed 2026-10-15).** [makerspet/oomwoo](https://github.com/makerspet/oomwoo) (Apache-2.0) is an open-source DIY robot vacuum (ROS 2 / Nav2, Raspberry Pi or old smartphone, STM32 I/O board), still in simulation, not buildable. Nothing to reuse in Phase 1 (our stack is Mac + ESP32 camera, Ackermann steering, no range sensors). For Phase 2: cheap vacuum spare-part 2D LiDARs (LDS02RR class, ~US$15–30, BOM sourcing tips) with the [kaiaai/LDS](https://github.com/kaiaai/LDS) driver library (ESP32 Arduino); Roomba-style cliff sensors and bumper micro-switches (~US$2) as hardware stop reflexes; their `recovery-safety`, `obstacle-avoidance` (front ToF) and `health-monitor` RFCs as design notes; `mac-dev-env` (pixi ROS 2 on Apple Silicon) for running SLAM/Nav2 on the Mac.

**Phase 2 item — remote access (added 2026-10-16).** Goal: open the dashboard from anywhere. Plan: Tailscale (private network between the owner's own devices; nothing exposed publicly) on the dashboard host, the Raspberry Pi once the stack moves off the Mac (Phase 2 onboard compute). Keep the dashboard password login on Tailscale connections. Not router port forwarding or a public tunnel, since the page can drive the rover. Remote use is mainly watching and STOP: mobile-data lag makes hold-to-drive overshoot; a dropped connection stops a hold within 0.35 s and the arm lease within 3 s. Until then the dashboard is home-network only (http://rover.local:8010/).

**Phase 2 study item — Wi-Fi motion zones (added 2026-10-16).** Explore [WaveSight](https://github.com/ErfanDL/WaveSight) (ESP32 Wi-Fi CSI presence/movement sensing, no camera) on separate ESP32 boards, one per room, as hints for where the dog is when the camera can't see him: the Mac polls each board's status; motion in a zone gives the search a target room or wakes the rover from parked, and the camera takes over once he is in view. Needs its own ESP32 per zone (~US$5–14; the rover's XIAO streams video, and a moving rover disturbs the channel itself). Limits: detects any motion, not the dog specifically (most useful when nobody else is home); a dog disturbs the signal far less than a person, and a sleeping dog may not register; zone only, no direction. A WaveSight port to the XIAO ESP32-S3 was built on 2026-10-13 (`~/Documents/Projects/WaveSight`, status LED on GPIO21; outside this repo, uncommitted). First test, free: temporarily flash that build to the XIAO, check whether it registers the dog walking between it and the router, then reflash the camera firmware.

**Phase 2 study item — collar tracker (added 2026-10-14).** Goal: know where the dog is without searching by camera. Options, cheapest first:
1. Owned AirTag on his collar (free): the rover hears its BLE advertisements (XIAO ESP32-S3 or Mac Bluetooth) and uses RSSI for near/far, which room, and hotter/colder search. Limits: no UWB direction (Apple restricts precision finding to iPhone), rotating address/keys so identity is "the only/strongest AirTag", unknown advertising behaviour while near the owner's iPhone, noisy indoor RSSI with body shadowing, and possible separation beeping if the owner is away for long.
2. Plain BLE beacon tag (~US$10–15): same RSSI information but a fixed ID and a configurable advertising rate.
3. UWB tag + rover UWB module (e.g. Qorvo DWM3000, ~US$50–100): ~10 cm ranging, and bearing with a two-antenna angle-of-arrival module, so the rover can head straight to him.
First step (free, no firmware change): log AirTag RSSI from the Mac with the tag on his collar, with him at the desk, across the room, and in another room, to check it advertises reliably at home and whether RSSI separates rooms. Use any tracker as a coarse hint that hands over to the camera once he is in view.

## Integration gate

Note (2026-10-14): the build is the OpenFrame Two 4WD, so the guide findings below (OpenFrame One) must be re-checked against the Two's guide. The OpenFrame One assembly guide V1.0 has been reviewed (materials page 3 and electronics pages 25-27). Its photographs show two drive motors labeled M1/M2, a steering servo, battery, controller assembly, and lighting connections labeled LED1/LED2. Exact component models, electrical ratings, and an autonomous command interface are not established by this guide. Before implementing the selected Mac-to-transmitter integration:

1. Record the exact controller, motors, steering actuator, battery, connectors, and original wiring.
2. Verify how Mac software can send throttle, steering, reverse, and stop commands to the transmitter over USB. USB connection alone does not establish a driving-command API.
3. Establish whether the interface requires changes to the original firmware. Any departure from the agreed original electronics must be discussed before proceeding.
4. Identify available feedback: battery voltage, wheel speed, steering position, and controller status. Do not assume any of these exist.
5. Verify stopping on Mac process failure, USB disconnection, transmitter/receiver link loss, and stale camera/sensor telemetry. Determine which timeouts the CyberBrick system enforces independently of the Mac. Measure stop latency.

If the existing electronics cannot accept autonomous commands, resolve that blocker before proceeding with navigation development.

## Architecture

- **Drive command path:** Mac navigation/control software -> USB -> CyberBrick transmitter -> existing wireless link -> CyberBrick receiver -> original motors and steering.
- **Perception path:** ESP32-S3 camera and added sensors -> Wi-Fi -> Mac perception/navigation software. Timestamp telemetry so the Mac can detect stale observations.
- **ESP32 role:** camera and sensor board only. Board is the XIAO ESP32S3 Sense (camera + mic); firmware is ESP-IDF (`firmware/xiao-cam/`), updated over Wi-Fi after the first USB flash. Powered by its own iPhone Air battery pack over USB-C, separate from the drive battery (decided 2026-10-14), so motor transients cannot reset it. Pin allocation for added sensors remains undecided. No ESP32-to-drive-controller connection is required by the current architecture.
- **Mac role:** perception, localization, patrol planning, rover operating states, and generation of drive commands.
- **Safety behavior:** bounded command lifetime, fault stop, explicit autonomous enable, and an accessible stop mechanism. The Mac should stop issuing motion when observations are stale; an independent CyberBrick command/link timeout must handle loss of the Mac or USB connection. These behaviors require verification.
- **Navigation compute:** the existing Mac runs navigation for the current build. Onboard compute is deferred because of cost; no companion computer purchase is in scope. Self-contained onboard operation remains a future goal.
- **Future migration:** keep the command/telemetry interface modular so onboard compute can be revisited later. Do not add cost or hardware solely to accommodate that upgrade now.
- **Additional sensors:** select after defining the mission. Distance sensing is proposed for obstacle detection; cliff sensing is required before operation near accessible stairs. Encoders and an IMU are candidates for navigation, subject to compatibility with the original base.

The camera alone should not be assumed to provide reliable obstacle distance, localization, or stair detection. Navigation runs on the Mac; ESP32 navigation is outside the current scope.

## Reuse from Mini-T (verified 2026-10-07)

Proven in https://github.com/ameyarya/ai-driven-mini-t — adapt, don't rebuild:
- Mac → USB CyberBrick transmitter drive commands (hold-to-drive, release-to-stop, per-side compensation). Transfers to the rover's USB/transmitter side; stock receiver + link timeout behavior still needs rover-side verification.
- Python move → stop → observe autonomy loop with adaptive timing, stale-frame checks, bounded commands, target-loss recovery, and step caps.
- Detector + dashboard pattern (labeled exact-planner-view image, frame age, decision status) and calibration files from day one. Ported 2026-10-14 as infrastructure: `mac/` has the XIAO link (frame-age measurement, 500 ms stale gate), the detector service slot, and a dashboard with the exact planner view; see `mac/README.md`. The Mini-T targets (can, bullseye) are not rover goals.
- Receiver-side watchdog + bounded commands (adopt the pattern for rover safety states; the signed OTA mechanism itself is Mini-T-specific).
- Not transferable: DJI/Mimo/MediaMTX camera pipeline (rover uses XIAO MJPEG/WebSocket), tracked-motion calibration (rover is wheeled), perfect-map simulation avoidance (real sensing is pending on both projects).

## Mechanical and electrical work

- Chassis hardware for the OpenFrame Two 4WD (checked 2026-10-14): 2 × 608 bearings (8 × 22 × 7 mm) and 4 × 625 bearings (5 × 16 × 5 mm), per the bearings modelled in the STEP and the print file. The print file's plate 7 "plastic baring replacement" has printable stand-ins (2 × 22 × 7 mm, 4 × 16 × 5 mm), so metal bearings are optional: build with the printed ones and order metal (packs of 10) if hubs wobble, drag or wear. (The designer's "8 × 2 × 7" is a typo for 8 × 22 × 7.) Screws: M2.5 button-head machine screws into plain 2.5 mm holes (CyberBrick convention per a Bambu forum report; the top-frame holes measure 2.5 mm). Housing mounts with M2.5 × 8–10 mm. Housing (pod mount + lid) printed in PETG (2026-10-15) had several fit issues; switched (2026-10-15) to Seeed's official XIAO ESP32S3 Sense case (two-piece snap case, kept local, not in git), printed as-is as a proof of concept. The case fits (2026-10-15). Mount (2026-10-15): `housing/` camera dock, the 4 × M2.5 front-grid plate with a 30 mm snap-in clip holding a 90° magnetic USB cable (Terasako 540° rotating, 6.53 mm barrel) pointing up. The magnetic tip stays in the XIAO's USB-C port; the case snaps onto the cable head (magnet checked strong, powers the XIAO) and the rotating elbow sets the tilt, which replaces the earlier Apple-plug collar idea. The cable is likely power-only: OTA over Wi-Fi is unaffected; USB recovery flashing needs a normal cable. Watch `reset_reason`/uptime for power drops on bumps. Print order (2026-10-15): chassis first, then check the screw holes and clip with `cam-dock-fit-test.stl` on the printed frame, then print the dock. Original pod rationale: the XIAO runs warm in a closed pod and PLA softens near 55–60 °C; frame and steering parts stay PLA for stiffness.

- Measure mounting space, payload allowance, steering clearance, turning radius, and ground clearance.
- Design a removable top mount with a useful camera view, antenna clearance, cable strain relief, and access to the original electronics.
- Design a printed housing for the XIAO Sense with a clear camera view and USB access for recovery flashing (normal updates are over Wi-Fi). The external u.FL antenna is required (the XIAO ESP32S3 has no onboard antenna): mount it high on a plastic wall, away from motors, batteries, and metal, without folding it; strain-relieve the cable because u.FL connectors tolerate few mating cycles. Check telemetry `rssi` across rooms after mounting.
- XIAO power: own iPhone Air battery pack over USB-C, isolated from the drive battery (2026-10-14). Camera mount is the front camera dock only (`housing/`, 4 × M2.5 into the frame's front grid rows; the XIAO case snaps onto a magnetic USB cable held upright); the power bank is strapped to the chassis separately, behind the steering-gear zone (>~6 cm from the frame front) so it does not press on the gear that rises 0.4 mm above the frame top. Mount the pack so the USB-C cable has strain relief, and watch telemetry `reset_reason` for `brownout` during rover tests.
- Select regulation and wiring based on measured loads and motor transients; do not assume spare power outputs are suitable.
- Test for resets and image/control failures during motor startup, steering, and reversing.
- Preserve room for selected sensors without obstructing motion or the camera.

## Milestones and completion checks

Milestones 0–2 make up Phase 1 (Mini-T parity); milestones 3–4 follow Phase 2 and are not yet scheduled.

### 0. Verify the base and select the mission (Phase 1)

Deliver a confirmed parts list, wiring reference, control-interface decision, and first-release mission. Complete when autonomous throttle/steering control is demonstrably feasible with the agreed electronics.

### 1. Drive and see (Phase 1)

Implement Mac-to-transmitter USB driving, mount the ESP32 camera/sensor board, and implement Wi-Fi telemetry and manual commissioning controls. Complete when steering, forward/reverse, and stop work repeatably while camera capture is active, without supply-related resets. Test initially with wheels lifted, then on the floor at low speed.

### 2. Safe autonomous roaming (Phase 1)

Implement idle, manual, autonomous, and fault states; add the selected obstacle sensors and bounded motion commands. Complete when the rover can roam in the test room and reliably stop for representative obstacles, stale commands, and sensor faults. Validate reset and communication-loss behavior. Record observed failures and test conditions.

### 3. Mission-specific navigation (after Phase 2)

Implement the selected behavior, such as visiting marked locations, following a person, or room-to-room patrol. Choose localization and compute architecture using prototype results. Complete when repeated runs meet agreed success criteria across the intended lighting, surfaces, and route conditions.

### 4. Unattended operation (after Phase 2)

Add recovery behavior, battery-aware stopping, and scheduling if required. Treat return-to-base and charging as separate development tasks. Complete only after endurance runs and fault tests demonstrate the selected operating envelope.

## Open decisions

| Decision | Status |
| --- | --- |
| Primary mission | Confirmed: home patrol; one-room roaming is the first autonomy milestone. Phase 1 mission (2026-10-14): dog companion, see "Phase 1 vision target" |
| Compute location | Locked (2026-10-06): Mac is the Phase 1 brain — no new purchases yet. Offboard architecture stands (rover only needs drive instructions), so a desk Pi 5 stays an approved later migration. Local LLM is out of Phase 1 scope for the same reason |
| Exact ESP32-S3 board and camera | Bench-verified 2026-10-14 (USB power): Seeed Studio XIAO ESP32S3 Sense, camera is an OV3660 (not OV2640; its native output is mirrored left-right, corrected by the firmware default `hmirror=1` since 2026-10-14), 8 MB octal PSRAM OK, Wi-Fi STA OK (RSSI -43 dBm at the desk). Mic verified (quiet room about -65 dBFS, responds to sound). Runs from the iPhone Air battery pack. Spare sensor pins still to budget |
| Existing electronics command interface | Selected: Mac USB -> owned CyberBrick transmitter -> wireless receiver. Verify USB command API and timeout behavior |
| Firmware framework and command transport | Transport selected (2026-10-06): WebSocket JSON telemetry + MJPEG video, timestamped for stale detection. Framework locked (2026-10-14): ESP-IDF v5.4.4 with esp32-camera, already proven on this Mac by a separate ESP32-S3 POC. Implemented in `firmware/xiao-cam/`: MJPEG on :81/stream (VGA, ~15 fps measured) with per-frame X-Timestamp-Us/X-Frame-Seq, JSON telemetry on ws://:80/ws at 5 Hz, SNTP epoch time, mDNS rover-cam.local |
| Floors, thresholds, stairs, pets, and lighting | Environment details needed. One dog lives here and is the Phase 1 subject: slow speeds, standoff distance, back off if he charges the rover |
| Budget and hardware already owned | Locked (2026-10-06): no $100+ spend yet — Phase 1 uses owned hardware (Mac + a dedicated second CyberBrick set) plus the XIAO ESP32S3 Sense (~US$14, bought 2026-10-07 as a small exception). Desk Pi and other Phase 2 purchases deferred |
| Obstacle sensors and motion feedback | Proposal (2026-10-07, from tank lesson): camera-only positioning forces stop-observe speeds (~4-5 s/step on the tank), so shortlist wheel encoders + IMU for when spending resumes. Select after base verification + environment details |
| Speed limit, runtime, stop distance, and mission success targets | Set during commissioning |
| Drive-electronics sharing with tank | Updated (2026-10-14): the Mini-T is retired as a POC. Drive path uses the Mini-T USB transmitter (20:6E:F1:45:0F:1C) and the second kit's receiver (20:6E:F1:4D:8D:04), which already runs the Mini-T signed wireless app service from an earlier truck experiment. `drive/rover_drive_app.py` (W/A/D forward with steering, S reverse, X stop+centre) installed over the air as rover-drive-0.1; radio link and signing key verified; steering on servo port S2 (rover-drive-0.2). Bench: motors run, stops ACK in ~0.3 s, receiver watchdog stopped the motors after a Mac SIGKILL. Motor signs and steering centre/travel to calibrate on the assembled chassis; USB-unplug and transmitter power-loss stop tests postponed until the chassis is assembled (2026-10-14). Steering servo on S2 is now the Mini-T launcher's positional 180° CyberBrick servo (the first one tried was continuous 360°, unusable); verified ±20° deflection and re-centring. Receiver verified fully wireless (off USB, own power): status and pulses work |
| Phase 1 vision target | Decided (2026-10-14, replacing "none" from earlier the same day): the dog (only dog in the house). Detection: Ultralytics YOLO11s COCO `dog` class. Activity: local Qwen3-VL-4B via Ollama describes what he is doing. Mini-T reuse stays infrastructure only, not its can/bullseye targets. Implemented camera-only in `mac/dog_watcher.py` + `mac/dog_behavior.py`; decisions are logged as intent until the drive path exists |

## Repository maintenance

- Keep confirmed facts separate from proposals and unresolved questions.
- Update this plan when scope, architecture, or milestone status changes.
- Record board versions, pin assignments, wiring, power requirements, and reproducible build/flash instructions as they become known.
- Keep firmware and mechanical source files versioned; document calibration and hardware test results alongside changes.
- Use focused commits and reviewable changes. Record hardware verification honestly; software checks do not establish physical rover safety or reliability.

## Next actions

1. Print and assemble the OpenFrame Two 4WD chassis (printing started 2026-10-14: first tire in TPU). Pull the MakerWorld electronics list first so missing fasteners, bearings, or drive components are ordered before assembly day.
2. Obtain and inspect the MakerWorld electronics list and wiring instructions.
3. Confirm the autonomous command interface while preserving the original drive electronics.
4. Define the first patrol route. The camera half of the home-computer interface is done (`mac/xiao_link.py`, 2026-10-14: frames ~55 ms old p50 at ~27 fps, stale gate 500 ms); the drive half waits on the integration gate.
5. XIAO firmware complete for Phase 1 bench work 2026-10-14 (see `firmware/xiao-cam/README.md`): OTA, runtime camera settings, health telemetry, mic level. Next: printed housing for XIAO + battery pack, then set `vflip`/`hmirror` once mounted.
6. Build the drive-and-see prototype before committing to a navigation stack.
7. Phase 1 done-criterion (restated 2026-10-14): dog companion. Parked and watching; follows when the dog moves (keeps him centred at a 30–55%-of-frame standoff, backs off if he closes fast); parks once he is still for 5 min and the vision model says he is asleep; if he leaves view, scans for up to 60 s then parks; logs a timeline of what he does. Camera side done and tested. Watcher → drive connected 2026-10-14 (`drive/drive_server.py` with dashboard arm lease + STOP, `mac/drive_client.py` move → stop → observe); unit-tested, not yet run on the chassis. Car-steering update 2026-10-16: turns are forward arcs (back up straight first if the dog is too close), and searching rotates roughly in place by alternating a forward arc and a straight reverse toward the side he was last seen, since the receiver accepts only W/A/S/D/X (no reverse with steering). Dashboard rebuilt 2026-10-16 along Mini-T lines (`mac/dashboard.py` + `dashboard.html`): one command, services started from the header, hold-to-drive manual control (new drive-server `/hold`), dog timeline, camera health, XIAO restart alerts for power drops. Simulation added 2026-10-16 (`sim/`, MuJoCo, closed-loop runs of the real behaviour + drive client; [results](sim/RESULTS.md)): never drove into the dog in 36 runs; found that the rover gets wedged on door frames/furniture and keeps pushing (needs stuck detection) and that the 60 s search covers only ~270° (scan pairs turn ~13°). Served on the home network at http://rover.local:8010/ (mDNS; password login for other devices, none on the Mac; port 80 is used by another local app).

## Camera board shortlist

- Proposed budget candidate: [Seeed Studio XIAO ESP32S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html), manufacturer listing approximately US$13.99 before shipping/tax as checked 2026-10-06. Buy the Sense camera bundle, not the bare XIAO ESP32S3.
- Manufacturer documentation specifies 8 MB PSRAM and exposes UART and I2C pins: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
- Ordered 2026-10-07 (camera + mic bundle). On arrival, verify sensor pin requirements, power, and the camera supplied by the seller before final selection.

## Assembly guide findings

- (OpenFrame One; superseded by the OpenFrame Two 4WD build, re-check against its guide.) Source: user-provided OpenFrame One assembly guide V1.0, 35 pages. The source PDF is retained locally and is not copied into the repository.
- Pages 25-27 establish physical assembly and connector placement; they do not specify a UART pinout, external command protocol, voltage ratings, or motion feedback.
- Page 34 directs the builder to the official Bambu Lab remote, and page 35 shows throttle, steering, and lighting controls. These describe the original manual-control setup, not autonomous integration.
- CyberBrick publishes an official MicroPython controller application repository: https://github.com/CyberBrick-Official/CyberBrick_Controller_Core . Investigate a Mac USB command adapter on the transmitter while preserving the existing receiver/motor wiring. This is a candidate approach, not a verified capability for this assembled model.
- Continue to treat the XIAO ESP32S3 Sense as a camera-board candidate; the guide alone does not confirm electrical compatibility or a suitable power connection.
