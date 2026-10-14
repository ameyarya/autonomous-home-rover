# Autonomous Home Rover

Autonomous indoor home-patrol rover built on the CyberBrick OpenFrame One RC car.
An ESP32-S3 camera board handles vision/sensors over Wi-Fi; a Mac runs navigation
and sends drive commands via USB to the CyberBrick transmitter, which relays them
wirelessly to the stock receiver and drive electronics.

## Architecture

- **Drive path:** Mac nav software → USB → CyberBrick transmitter → wireless link → stock receiver → motors + steering.
- **Perception path:** XIAO ESP32S3 Sense camera/sensors → Wi-Fi → Mac (WebSocket JSON telemetry + MJPEG, timestamped for stale detection).
- **Brain:** offboard Mac for Phase 1 (a desk Pi is an approved later migration); no onboard compute.

## Status

- Chassis not yet printed — print and assembly slipped, not yet rescheduled (as of 2026-10-13).
- XIAO ESP32S3 Sense firmware configured and bench-verified 2026-10-14: OV3660 camera (VGA MJPEG ~15 fps), 5 Hz telemetry with health and mic level, settings over HTTP, updates over Wi-Fi ([firmware/xiao-cam](firmware/xiao-cam/README.md)). Powered by an iPhone Air battery pack.
- Autonomy software will be ported from the working Mini-T stack (command path, autonomy loop, detector + dashboard, calibration discipline) — see [PLAN.md](PLAN.md) for the reuse mapping.

## Docs

- [PLAN.md](PLAN.md) — goals, architecture, milestones, open decisions, next actions.
- [firmware/xiao-cam](firmware/xiao-cam/README.md) — XIAO camera/telemetry firmware (ESP-IDF): endpoints, build, flash.
- Related: [ai-driven-mini-t](https://github.com/ameyarya/ai-driven-mini-t) — the proven tank project this rover builds on.
