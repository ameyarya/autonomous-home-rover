# Housing (v1 draft)

Top housing for the rover: a base plate that screws onto the OpenFrame Two top frame, a cradle
for the iPhone Air MagSafe Battery, and a pod at the front for the XIAO ESP32S3 Sense with its
camera looking forward. Parametric: edit the numbers at the top of `housing.py` and re-run.

| File | Print |
| --- | --- |
| `out/housing-pod-testfit.stl` | **Print first** (~30 min): the pod on a 60 × 32.5 mm plate section that screws onto the frame with 4 screws (the two front rows). Check the XIAO fit, lens window, USB-C opening, lid, and camera height/angle on the car. |
| `out/housing-base.stl` | Plate + battery cradle + pod, 76 × 178 × 26 mm (fits the A1 mini bed). Print flat, no supports. |
| `out/housing-lid.stl` | Pod lid, printed roof-down. Stick the antenna on top of the roof inside the rim. |

## Design

- **Mounting:** 8 × M2.5 × 8–10 mm button-head screws into the frame's 2.5 mm hole grid (rows 50 mm
  apart, 22.5 mm pitch): the front row beside the pod (the pod is 28.8 mm wide, the rows 50 mm apart),
  the row just behind the pod, and two rows under the battery in counterbores. The plate overhangs
  the frame front by 2.5 mm so the front holes have 3 mm of material to the edge. Fit the front
  screws before the lid (they sit 3 mm outside the lid's roof).
- **Camera:** lens window in the pod front, camera tilted 5° up (`TILT_UP_DEG`). At ~10 cm off the
  floor, level-to-slightly-up covers a dog from paws to head at 1–2 m.
- **XIAO:** stack (24.4 × 17.8 × 14 mm from Seeed's STEP) stands upright, camera forward, 0.4 mm
  clearance. USB-C exits the left wall (as seen from behind), toward the battery's USB-C.
- **Antenna:** cable goes up through a slot in the lid; the 39 × 19 mm antenna sticks on the roof,
  the highest point, with only plastic around it.
- **Battery:** lies flat along the car in a 6 mm-walled cradle, USB-C edge forward with an opening in
  the front wall, 27.5 mm cable gap to the pod (use a short right-angle USB-C to USB-C cable).
  Two pairs of strap slots in the overhang for a Velcro strap or rubber band. Finger notch at the rear.
- **Vents** on the right wall and the lid (also let sound reach the mic).
- **Chassis clearance:** checked against the OpenFrame Two STEP: no collisions. A steering-gear part
  rises 0.4 mm above the frame top near the front; the plate has a 1.5 mm relief pocket over it.
  Tyre tops sit ~10 mm below the deck, so steering lock can't reach the plate.

## Checks

`check.py` (needs the local chassis STEP and Seeed's XIAO model, both kept out of git) verifies
zero overlap between the XIAO, battery, lid and base, and collisions with the chassis.

## Assumptions to confirm on the test-fit print

- Battery size 122.2 × 70.8 × 7.7 mm is from a teardown measurement, not Apple.
- The lens window position comes from Seeed's model; the OV3660 module on your board may sit
  slightly differently.
- Antenna size (39 × 19 mm) is from a forum report.
