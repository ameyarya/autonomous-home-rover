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

- Chassis not yet printed — print and assembly scheduled next week (as of 2026-10-07).
- XIAO ESP32S3 Sense (camera + mic) ordered.
- Autonomy software will be ported from the working Mini-T stack (command path, autonomy loop, detector + dashboard, calibration discipline) — see [PLAN.md](PLAN.md) for the reuse mapping.

## Docs

- [PLAN.md](PLAN.md) — goals, architecture, milestones, open decisions, next actions.
- Related: [ai-driven-mini-t](https://github.com/ameyarya/ai-driven-mini-t) — the proven tank project this rover builds on.
