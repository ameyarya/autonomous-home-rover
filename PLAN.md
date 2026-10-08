# Home Rover — Project Plan

Repository: https://github.com/ameyarya/autonomous-home-rover

Status: Chassis not yet printed — print and assembly scheduled next week (as of 2026-10-07). XIAO Sense ordered; autonomy software to be ported from the Mini-T project. This is a living plan; update it as hardware is verified and decisions are made.

## Goal

Build an autonomous indoor rover using the CyberBrick OpenFrame One RC car as the base, with an ESP32-S3 and camera mounted on top. The long-term goal is autonomous operation at home. The primary mission is home patrol: visit designated locations and capture camera observations. The navigation method, patrol schedule, and reporting behavior remain to be selected.

Base project: https://makerworld.com/en/models/2243313-cyberbrick-openframe-one-rc-car#profileId-2442179

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

## Integration gate

The OpenFrame One assembly guide V1.0 has been reviewed (materials page 3 and electronics pages 25-27). Its photographs show two drive motors labeled M1/M2, a steering servo, battery, controller assembly, and lighting connections labeled LED1/LED2. Exact component models, electrical ratings, and an autonomous command interface are not established by this guide. Before implementing the selected Mac-to-transmitter integration:

1. Record the exact controller, motors, steering actuator, battery, connectors, and original wiring.
2. Verify how Mac software can send throttle, steering, reverse, and stop commands to the transmitter over USB. USB connection alone does not establish a driving-command API.
3. Establish whether the interface requires changes to the original firmware. Any departure from the agreed original electronics must be discussed before proceeding.
4. Identify available feedback: battery voltage, wheel speed, steering position, and controller status. Do not assume any of these exist.
5. Verify stopping on Mac process failure, USB disconnection, transmitter/receiver link loss, and stale camera/sensor telemetry. Determine which timeouts the CyberBrick system enforces independently of the Mac. Measure stop latency.

If the existing electronics cannot accept autonomous commands, resolve that blocker before proceeding with navigation development.

## Architecture

- **Drive command path:** Mac navigation/control software -> USB -> CyberBrick transmitter -> existing wireless link -> CyberBrick receiver -> original motors and steering.
- **Perception path:** ESP32-S3 camera and added sensors -> Wi-Fi -> Mac perception/navigation software. Timestamp telemetry so the Mac can detect stale observations.
- **ESP32 role:** camera and sensor board only. Exact board, camera, pin allocation, power supply, and firmware framework remain undecided. No ESP32-to-drive-controller connection is required by the current architecture.
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
- Detector + dashboard pattern (labeled exact-planner-view image, frame age, decision status) and calibration files from day one.
- Receiver-side watchdog + bounded commands (adopt the pattern for rover safety states; the signed OTA mechanism itself is Mini-T-specific).
- Not transferable: DJI/Mimo/MediaMTX camera pipeline (rover uses XIAO MJPEG/WebSocket), tracked-motion calibration (rover is wheeled), perfect-map simulation avoidance (real sensing is pending on both projects).

## Mechanical and electrical work

- Measure mounting space, payload allowance, steering clearance, turning radius, and ground clearance.
- Design a removable top mount with a useful camera view, antenna clearance, cable strain relief, and access to the original electronics.
- Verify supply voltages and spare current capacity before powering the ESP32 from the original battery system.
- Select regulation and wiring based on measured loads and motor transients; do not assume spare power outputs are suitable.
- Test for resets and image/control failures during motor startup, steering, and reversing.
- Preserve room for selected sensors without obstructing motion or the camera.

## Milestones and completion checks

### 0. Verify the base and select the mission

Deliver a confirmed parts list, wiring reference, control-interface decision, and first-release mission. Complete when autonomous throttle/steering control is demonstrably feasible with the agreed electronics.

### 1. Drive and see

Implement Mac-to-transmitter USB driving, mount the ESP32 camera/sensor board, and implement Wi-Fi telemetry and manual commissioning controls. Complete when steering, forward/reverse, and stop work repeatably while camera capture is active, without supply-related resets. Test initially with wheels lifted, then on the floor at low speed.

### 2. Safe autonomous roaming

Implement idle, manual, autonomous, and fault states; add the selected obstacle sensors and bounded motion commands. Complete when the rover can roam in the test room and reliably stop for representative obstacles, stale commands, and sensor faults. Validate reset and communication-loss behavior. Record observed failures and test conditions.

### 3. Mission-specific navigation

Implement the selected behavior, such as visiting marked locations, following a person, or room-to-room patrol. Choose localization and compute architecture using prototype results. Complete when repeated runs meet agreed success criteria across the intended lighting, surfaces, and route conditions.

### 4. Unattended operation

Add recovery behavior, battery-aware stopping, and scheduling if required. Treat return-to-base and charging as separate development tasks. Complete only after endurance runs and fault tests demonstrate the selected operating envelope.

## Open decisions

| Decision | Status |
| --- | --- |
| Primary mission | Confirmed: home patrol; one-room roaming is the first autonomy milestone |
| Compute location | Locked (2026-10-06): Mac is the Phase 1 brain — no new purchases yet. Offboard architecture stands (rover only needs drive instructions), so a desk Pi 5 stays an approved later migration. Local LLM is out of Phase 1 scope for the same reason |
| Exact ESP32-S3 board and camera | Ordered (2026-10-07): Seeed Studio XIAO ESP32S3 Sense bundle (camera + mic). On arrival: verify sensor pin requirements, power, and supplied camera before final selection; create pin/power budget |
| Existing electronics command interface | Selected: Mac USB -> owned CyberBrick transmitter -> wireless receiver. Verify USB command API and timeout behavior |
| Firmware framework and command transport | Transport selected (2026-10-06): WebSocket JSON telemetry + MJPEG video, timestamped for stale detection. Framework pending: user rejected Arduino/ESP-IDF/MicroPython; awaiting clarification |
| Floors, thresholds, stairs, pets, and lighting | Environment details needed |
| Budget and hardware already owned | Locked (2026-10-06): no $100+ spend yet — Phase 1 uses owned hardware only (Mac + CyberBrick kit). ESP32 camera board and desk Pi purchases deferred to later milestones |
| Obstacle sensors and motion feedback | Proposal (2026-10-07, from tank lesson): camera-only positioning forces stop-observe speeds (~4-5 s/step on the tank), so shortlist wheel encoders + IMU for when spending resumes. Select after base verification + environment details |
| Speed limit, runtime, stop distance, and mission success targets | Set during commissioning |
| Drive-electronics sharing with tank | Open (2026-10-07): the owned CyberBrick kit currently serves the working Mini-T — decide whether the rover build reuses it or a second set is bought when spending resumes |

## Repository maintenance

- Keep confirmed facts separate from proposals and unresolved questions.
- Update this plan when scope, architecture, or milestone status changes.
- Record board versions, pin assignments, wiring, power requirements, and reproducible build/flash instructions as they become known.
- Keep firmware and mechanical source files versioned; document calibration and hardware test results alongside changes.
- Use focused commits and reviewable changes. Record hardware verification honestly; software checks do not establish physical rover safety or reliability.

## Next actions

1. Print and assemble the OpenFrame One chassis (scheduled next week as of 2026-10-07). Pull the MakerWorld electronics list first so missing fasteners, bearings, or drive components are ordered before assembly day.
2. Obtain and inspect the MakerWorld electronics list and wiring instructions.
3. Confirm the autonomous command interface while preserving the original drive electronics.
4. Define the first patrol route and the home-computer software/transport interface.
5. On XIAO arrival: verify camera/mic/pins/power and create the pin/power budget (board already ordered).
6. Build the drive-and-see prototype before committing to a navigation stack.

## Camera board shortlist

- Proposed budget candidate: [Seeed Studio XIAO ESP32S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html), manufacturer listing approximately US$13.99 before shipping/tax as checked 2026-10-06. Buy the Sense camera bundle, not the bare XIAO ESP32S3.
- Manufacturer documentation specifies 8 MB PSRAM and exposes UART and I2C pins: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
- Ordered 2026-10-07 (camera + mic bundle). On arrival, verify sensor pin requirements, power, and the camera supplied by the seller before final selection.

## Assembly guide findings

- Source: user-provided OpenFrame One assembly guide V1.0, 35 pages. The source PDF is retained locally and is not copied into the repository.
- Pages 25-27 establish physical assembly and connector placement; they do not specify a UART pinout, external command protocol, voltage ratings, or motion feedback.
- Page 34 directs the builder to the official Bambu Lab remote, and page 35 shows throttle, steering, and lighting controls. These describe the original manual-control setup, not autonomous integration.
- CyberBrick publishes an official MicroPython controller application repository: https://github.com/CyberBrick-Official/CyberBrick_Controller_Core . Investigate a Mac USB command adapter on the transmitter while preserving the existing receiver/motor wiring. This is a candidate approach, not a verified capability for this assembled model.
- Continue to treat the XIAO ESP32S3 Sense as a camera-board candidate; the guide alone does not confirm electrical compatibility or a suitable power connection.
