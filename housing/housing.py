"""Rover camera dock: a plate that screws onto the OpenFrame Two top frame with an upright clip
that holds the barrel of a 90° magnetic USB cable pointing up. The XIAO ESP32S3 Sense, in Seeed's
official case with the cable's magnetic tip in its USB-C port, snaps magnetically onto the cable
head; the cable's rotating elbow sets the camera tilt. Pull it off to remove.

Run: cad/.venv/bin/python housing/housing.py   -> housing/out/cam-dock.stl, cam-dock-fit-test.stl
housing/check.py checks for collisions against the local chassis STEP.

World coordinates are the chassis STEP's: X across the car, Y along it (front = -Y),
Z up. All dimensions in mm. Edit the parameters below and re-run.
"""
from pathlib import Path

from build123d import Align, Box, Cylinder, Pos, export_stl

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'housing/out'
CHASSIS_STEP = ROOT / 'OpenFrame One 4W Final.step'           # local only, never committed

# ---- chassis interface (measured from the OpenFrame Two STEP) ----
FRAME_TOP_Z = 40.4
FRAME_FRONT_Y = -172.1
GRID_X = (-28.7, 21.3)                 # two rows of 2.5 mm holes, 50 mm apart
GRID_Y = [-170.1 + 22.5 * i for i in range(9)]
CX = sum(GRID_X) / 2                   # centreline of the hole grid
# Steering gear that rises 0.43 mm above the frame top: pocket in the plate underside.
RELIEFS = [((-15.5, -10.5), (-147.3, -119.6), 1.5)]   # (x range, y range, depth)
RELIEF_MARGIN = 1.5

# ---- parts ----
BARREL_D = 6.53                        # magnetic cable barrel below the elbow (measured)

# ---- design parameters ----
PLATE_T = 3.0
PLATE_FRONT_EXTEND = 2.5               # overhang past the frame front: ~3 mm of material at the front screws
SCREW_ROWS = 2                         # front grid rows used (4 screws)
CLIP_H = 30.0                          # clip height above the plate; the elbow rests on its top
BORE_D = 6.7                           # snug on the barrel
CLIP_WALL = 2.4
CLIP_MOUTH = 5.4                       # opening at the back, narrower than the barrel: it snaps in
FOOT = (18.0, 16.0, 6.0)               # wider base so the clip doesn't snap off at the plate
CLIP_Y = FRAME_FRONT_Y + 9.0           # clip axis, behind the frame front
TEST_MOUTHS = (5.0, 5.4, 5.8)          # fit test: one short clip per mouth width, marked by 1/2/3 notches
TEST_H = 10.0
SCREW_CLEAR_D = 2.9                    # M2.5 clearance
HEAD_D = 5.4                           # M2.5 button head, for clearance checks

PLATE_TOP = FRAME_TOP_Z + PLATE_T


def clip():
    """Upright C-clip in its own frame: axis on Z, origin at the plate top, mouth toward +Y (back)."""
    od = BORE_D + 2 * CLIP_WALL
    body = Cylinder(od / 2, CLIP_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    fx, fy, fz = FOOT
    body += Pos(0, 0, fz / 2) * Box(fx, fy, fz)
    body -= Pos(0, 0, -1) * Cylinder(BORE_D / 2, CLIP_H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Mouth: full height at the back, so the barrel snaps in and the cable bends out anywhere.
    body -= Pos(0, max(od, fy) / 2, CLIP_H / 2) * Box(CLIP_MOUTH, max(od, fy), CLIP_H + 2)
    return body, od


def fit_test():
    """Short clips side by side on a strip, one per TEST_MOUTHS entry, to pick the snap fit
    before printing the full dock. Notches on the strip's front edge: 1 = first mouth, etc."""
    od = BORE_D + 2 * CLIP_WALL
    pitch = od + 6
    n = len(TEST_MOUTHS)
    strip = Pos(0, 0, 1) * Box(pitch * n, od + 6, 2)
    for i, mouth in enumerate(TEST_MOUTHS):
        x = (i - (n - 1) / 2) * pitch
        ring = Cylinder(od / 2, TEST_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        strip += Pos(x, 0, 0) * ring
        strip -= Pos(x, 0, -1) * Cylinder(BORE_D / 2, TEST_H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        strip -= Pos(x, od / 2 + 3, TEST_H / 2) * Box(mouth, od + 6, TEST_H + 4)
        for k in range(i + 1):
            strip -= Pos(x + (k - i / 2) * 2.0, -(od + 6) / 2, 1) * Box(1.0, 2.0, 4)
    return strip


def build():
    c, od = clip()
    rows = GRID_Y[:SCREW_ROWS]
    plate_front = FRAME_FRONT_Y - PLATE_FRONT_EXTEND
    plate_len = rows[-1] + 5.5 - plate_front
    plate_w = GRID_X[1] - GRID_X[0] + 10
    plate = Pos(CX, plate_front + plate_len / 2, FRAME_TOP_Z + PLATE_T / 2) * Box(plate_w, plate_len, PLATE_T)
    dock = plate + Pos(CX, CLIP_Y, PLATE_TOP) * c

    for (x0, x1), (y0, y1), depth in RELIEFS:
        dock -= Pos((x0 + x1) / 2, (y0 + y1) / 2, FRAME_TOP_Z + depth / 2 - 0.01) * Box(
            x1 - x0 + 2 * RELIEF_MARGIN, y1 - y0 + 2 * RELIEF_MARGIN, depth)

    assert all(abs(x - CX) > FOOT[0] / 2 + HEAD_D / 2 for x in GRID_X), 'screws would hit the clip'
    for y in rows:
        for x in GRID_X:
            dock -= Pos(x, y, FRAME_TOP_Z + PLATE_T / 2) * Cylinder(SCREW_CLEAR_D / 2, PLATE_T * 3)

    report = {
        'plate_mm': (round(plate_w, 1), round(plate_len, 1), PLATE_T),
        'clip_od_bore_mouth': (round(od, 1), BORE_D, CLIP_MOUTH),
        'clip_top_above_frame': round(PLATE_T + CLIP_H, 1),
        'screw_rows_y': [round(y, 1) for y in rows],
    }
    return dock, report


if __name__ == '__main__':
    dock, report = build()
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob('*.stl'):
        old.unlink()
    export_stl(dock, str(OUT / 'cam-dock.stl'))
    export_stl(fit_test(), str(OUT / 'cam-dock-fit-test.stl'))
    for k, v in report.items():
        print(f'{k}: {v}')
    bb = dock.bounding_box()
    print(f'cam dock print size: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, volume {dock.volume / 1000:.1f} cm3')
