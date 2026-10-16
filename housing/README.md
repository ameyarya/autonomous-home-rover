# Camera dock

A dock for the XIAO ESP32S3 Sense in Seeed's official case (kept local, not in git) at the front
of the OpenFrame Two: a plate that screws onto the top frame with an upright clip that holds a 90°
magnetic USB cable (Terasako 540° rotating, braided) with its barrel pointing up. The cable's
magnetic tip stays in the XIAO's USB-C port; the case snaps onto the cable head and the rotating
elbow sets the camera tilt. Pull the case off to remove it. Power comes from the iPhone Air power
bank strapped to the chassis. Parametric: edit the numbers at the top of `housing.py` and re-run.

| File | Print |
| --- | --- |
| `out/cam-dock-fit-test.stl` | Print first (~15 min): a 1.2 mm copy of the base plate (same outline, screw holes and gear relief) with three 10 mm clip slices, mouths 5.0 / 5.4 / 5.8 mm, marked by 1 / 2 / 3 notches on the front edge. Check all 4 holes line up with the frame grid (M2.5 screws drop through), then snap the barrel into each clip and pick the one that clicks in with firm thumb pressure and doesn't spin. |
| `out/cam-dock.stl` | 60 × 32.5 × 3 mm plate + 30 mm clip, 33 mm tall overall. Plate on the bed, no supports, PETG. ~9 g. Set `CLIP_MOUTH` to the fit test's winner first. |

## Design

- **Mounting:** 4 × M2.5 × 8–10 mm button-head screws into the frame's 2.5 mm hole grid (rows 50 mm
  apart), the front row and the row behind it. The plate overhangs the frame front by 2.5 mm.
- **Clip:** 6.7 mm bore for the 6.53 mm cable barrel (measured), 2.4 mm walls, 30 mm tall, on an
  18 × 16 × 6 mm foot. A 5.4 mm mouth runs the full height at the back: push the barrel in from
  behind and it snaps in (PETG flexes); the braided cable bends out through the mouth and runs
  back along the plate. The elbow rests on the top of the clip.
- **Camera:** on the cable head, about 33 mm + elbow above the frame; tilt by rotating the elbow.
  The antenna stays on the case, pointing up.
- **Chassis clearance:** checked against the OpenFrame Two STEP, no collisions. The plate has a
  1.5 mm relief pocket over the steering gear that rises 0.4 mm above the frame top.

## Checks

`check.py` (needs the local chassis STEP, kept out of git) verifies no collisions with the chassis.

## To confirm on the first print

- Snap fit: chosen with the fit test. If none fits (all too loose or all too tight), change `BORE_D`.
- Magnet on bumps: after a drive test, check the XIAO's reset reason and uptime for power drops.
- Magnetic cables are usually power-only; recovery flashing over USB needs a normal cable.
