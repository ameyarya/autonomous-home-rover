# mac

Mac side of the rover camera pipeline: receives the XIAO's video and telemetry, measures frame
age, refuses stale frames, and feeds the detector ported from
[ai-driven-mini-t](https://github.com/ameyarya/ai-driven-mini-t). Nothing here sends drive commands.

| File | What |
| --- | --- |
| `xiao_link.py` | `XiaoLink`: background MJPEG + WebSocket readers, XIAO→Mac clock mapping, `fresh_frame()` stale gate, `save()` in Mini-T frame naming. `python3 xiao_link.py` prints live link stats. |
| `dashboard.py` | http://localhost:8010/: live view, link health, and the planner view (exact labeled frame the detector saw). |
| `rover_detector.py` | Mini-T detector (YOLO-World / bullseye matcher), ported with only paths and port (8767) changed. |
| `test_xiao_link.py` | Tests against a fake XIAO: clock mapping, frame age, stale detection, telemetry loss, camera fault, captured-after waits. |

Python 3 standard library only; the detector runs in its own environment (below).

## Run

```sh
python3 mac/dashboard.py                    # or --host 192.168.1.194 if mDNS is slow
python3 mac/xiao_link.py --seconds 10       # link stats only
cd mac && python3 -m unittest test_xiao_link
```

In the dashboard, type a target ("cup", "person", "bullseye target") and click **Observe**: it waits
for a frame captured after the click, saves it to `mac/vision-output/` (gitignored), runs the
detector, and shows the labeled result with the frame's age.

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
