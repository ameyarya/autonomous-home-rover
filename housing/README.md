# Camera pod mount

A pod for the XIAO ESP32S3 Sense at the front of the OpenFrame Two, on a small plate that screws
onto the top frame, plus a lid. The iPhone Air power bank is strapped to the chassis separately.
Parametric: edit the numbers at the top of `housing.py` and re-run.

| File | Print |
| --- | --- |
| `out/pod-mount.stl` | Pod + 60 × 32.5 × 3 mm plate, ~26 mm tall. Plate on the bed, no supports. 3 walls recommended (screw holes, lens window). |
| `out/pod-lid.stl` | Lid, flat roof on the bed, lip up; no supports. The antenna sticks on the flat roof. |

## Design

- **Mounting:** 4 × M2.5 × 8–10 mm button-head screws into the frame's 2.5 mm hole grid (rows 50 mm
  apart): the front row beside the pod and the row just behind it. The plate overhangs the frame
  front by 2.5 mm so the front holes have 3 mm of material to the edge. Fit the screws before the lid.
- **Camera:** lens window in the pod front, tilted 5° up (`TILT_UP_DEG`), lens ~10 cm off the floor.
- **XIAO:** stack (24.4 × 17.8 × 14 mm, from Seeed's STEP) stands upright with the camera forward, 0.4 mm
  clearance; the lid's lip holds it down. USB-C exits the left wall (seen from behind).
- **Antenna:** cable up through the slot in the lid; the 39 × 19 mm antenna sticks on the roof, the
  highest point, with only plastic around it.
- **Vents:** slots in the right wall and lid for heat and to let sound reach the mic (placement is
  an estimate; watch for resets or heat while streaming).
- **Chassis clearance:** checked against the OpenFrame Two STEP, no collisions. A steering gear rises
  0.4 mm above the frame top 25–55 mm behind the front; the plate has a 1.5 mm relief pocket over it.

**Power bank:** strap it behind the steering-gear zone (more than ~6 cm back from the front of the
frame) so it doesn't press on the gear, and use a short right-angle USB-C to USB-C cable to the pod.

## Checks

`check.py` (needs the local chassis STEP and Seeed's XIAO model, both kept out of git) verifies zero
overlap between the XIAO, mount and lid, and no collisions with the chassis.

## Assumptions to confirm on the first print

- The lens window position comes from Seeed's model; the OV3660 module on your board may sit
  slightly differently.
- Antenna size (39 × 19 mm) is from a forum report.
