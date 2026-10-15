# Camera dock

A dock for the XIAO ESP32S3 Sense in Seeed's official case (kept local, not in git) at the front
of the OpenFrame Two: a plate that screws onto the top frame with a tower that holds an Apple USB-C
cable plug pointing up. The case plugs onto it USB-C end down, antenna up, and drops into a collar;
lift it off to remove. Power comes from the iPhone Air power bank strapped to the chassis.
Parametric: edit the numbers at the top of `housing.py` and re-run.

| File | Print |
| --- | --- |
| `out/cam-dock.stl` | 60 × 32.5 × 3 mm plate + 26.8 × 11.4 × 29 mm tower, 32 mm tall overall. Plate on the bed, no supports, PETG. ~11 g. |

## Design

- **Mounting:** 4 × M2.5 × 8–10 mm button-head screws into the frame's 2.5 mm hole grid (rows 50 mm
  apart), the front row and the row behind it. The plate overhangs the frame front by 2.5 mm.
- **Plug channel:** 13.25 × 7.4 mm, open at the back for the full height: slide the plug in from
  behind; the cable bends out wherever the plug ends and runs back along the plate. Sized to Apple's
  USB-C cable envelope (its case guidance keeps plugs within 12.45 × 6.60 mm); no exact Apple plug
  dimensions are published.
- **Collar:** 22.8 × 7.0 mm, 5 mm deep, around the case's slim USB-C end (22.2 × ~6.4 mm for its
  first ~7 mm, measured from Seeed's STL). The case rests on ledges either side of the plug, so the
  collar takes bumps and the XIAO's USB-C port only carries power.
- **Camera:** faces forward (-Y), collar top 32 mm above the frame. `TILT_UP_DEG` tilts the tower
  (0 = level); a different angle is a reprint.
- **Antenna:** stays on the case, pointing up, the highest point on the rover.
- **Chassis clearance:** checked against the OpenFrame Two STEP, no collisions. The plate has a
  1.5 mm relief pocket over the steering gear that rises 0.4 mm above the frame top.

## Checks

`check.py` (needs the local chassis STEP, kept out of git) verifies no collisions with the chassis.

## Assumptions to confirm on the first print

- Case slim-end depth (6.4 mm) is read from Seeed's STL with the two halves assumed to stack flat;
  if the case is loose or tight in the collar, change `CASE_END`.
- Apple plug size is the published upper bound; if the plug is loose, it's held in line by the case
  once plugged in.
