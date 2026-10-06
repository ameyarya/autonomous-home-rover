# Home Rover — Project Plan

Repository: https://github.com/ameyarya/autonomous-home-rover

Status: Initial scope, 2026-10-06. This is a living plan; update it as hardware is verified and decisions are made.

## Goal

Build an autonomous indoor rover using the CyberBrick OpenFrame One RC car as the base, with an ESP32-S3 and camera mounted on top. The long-term goal is autonomous operation at home. The primary mission is home patrol: visit designated locations and capture camera observations. The navigation method, patrol schedule, and reporting behavior remain to be selected.

Base project: https://makerworld.com/en/models/2243313-cyberbrick-openframe-one-rc-car#profileId-2442179

## Confirmed scope

- Use the frame, drivetrain, and driving electronics exactly as specified by the MakerWorld project.
- Add an ESP32-S3 with a camera on top of the frame.
- Integrate the ESP32 with the existing driving electronics; do not assume replacement motor drivers or a different drive platform.
- Maintain this repository throughout the project, including this plan, hardware decisions, firmware, wiring, build notes, and validation results.

## Proposed first release

Start with slow autonomous roaming and obstacle avoidance in one controlled indoor room, with manual charging. This is the initial milestone toward home patrol; detailed acceptance targets remain to be set.

The first release should support manual commissioning, camera capture, autonomous start/stop, obstacle stopping, and a safe stopped state on faults. Room-to-room navigation, person following, scheduled patrols, mapping, and automatic docking are later candidates rather than committed first-release features.

## Integration gate

The MakerWorld parts list, controller documentation, and wiring have not yet been verified. Before selecting an integration method:

1. Record the exact controller, motors, steering actuator, battery, connectors, and original wiring.
2. Identify a documented or experimentally validated interface for throttle, steering, reverse, and stop.
3. Establish whether the interface requires changes to the original firmware. Any departure from the agreed original electronics must be discussed before proceeding.
4. Identify available feedback: battery voltage, wheel speed, steering position, and controller status. Do not assume any of these exist.
5. Verify that a lost ESP32 command or controller connection can bring the vehicle to a stop. Stop latency must be measured.

If the existing electronics cannot accept autonomous commands, resolve that blocker before proceeding with navigation development.

## Architecture

- **Original drive system:** executes throttle and steering commands through the verified interface.
- **ESP32-S3 + camera:** captures images, reads added sensors, coordinates rover states, and sends drive commands. Exact board, camera, pin allocation, and firmware framework are undecided.
- **Safety behavior:** bounded command lifetime, fault stop, explicit autonomous enable, and an accessible stop mechanism. Verify whether the original controller itself enforces command expiry; ESP32 software alone cannot cover an ESP32 failure.
- **Navigation compute:** phase 1 uses a home computer over Wi-Fi for perception and navigation. Later, move those functions onboard so patrol does not require a home computer or Wi-Fi connection. Keep the ESP32 responsible for local sensor handling and bounded drive commands through the verified original-controller interface. Onboard compute is deferred because of cost; no Raspberry Pi, Jetson, or other companion computer purchase is in the current scope. Benchmark the home-computer workload before revisiting a future migration.
- **Future migration:** keep the command/telemetry interface modular so onboard compute can be revisited later. Do not add cost or hardware solely to accommodate that upgrade now.
- **Additional sensors:** select after defining the mission. Distance sensing is proposed for obstacle detection; cliff sensing is required before operation near accessible stairs. Encoders and an IMU are candidates for navigation, subject to compatibility with the original base.

The camera alone should not be assumed to provide reliable obstacle distance, localization, or stair detection. Full home mapping and robust navigation on the S3 alone are unproven for this project.

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

Integrate the ESP32, mount the camera, and implement manual commissioning controls and telemetry. Complete when steering, forward/reverse, and stop work repeatably while camera capture is active, without supply-related resets. Test initially with wheels lifted, then on the floor at low speed.

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
| Compute location | Confirmed: home computer over Wi-Fi for the current build; onboard compute deferred due to cost, with self-contained operation retained as a future goal |
| Exact ESP32-S3 board and camera | Not selected |
| Existing electronics command interface | Must verify |
| Firmware framework and command transport | Select after interface verification |
| Floors, thresholds, stairs, pets, and lighting | Environment details needed |
| Budget and hardware already owned | User input needed |
| Obstacle sensors and motion feedback | Select after base verification |
| Speed limit, runtime, stop distance, and mission success targets | Set during commissioning |

## Repository maintenance

- Keep confirmed facts separate from proposals and unresolved questions.
- Update this plan when scope, architecture, or milestone status changes.
- Record board versions, pin assignments, wiring, power requirements, and reproducible build/flash instructions as they become known.
- Keep firmware and mechanical source files versioned; document calibration and hardware test results alongside changes.
- Use focused commits and reviewable changes. Record hardware verification honestly; software checks do not establish physical rover safety or reliability.

## Next actions

1. Obtain and inspect the MakerWorld electronics list and wiring instructions.
2. Confirm the autonomous command interface while preserving the original drive electronics.
3. Define the first patrol route and the home-computer software/transport interface.
4. Choose the ESP32-S3 camera board and create a pin/power budget.
5. Build the drive-and-see prototype before committing to a navigation stack.
