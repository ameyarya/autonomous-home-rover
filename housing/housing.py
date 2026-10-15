"""Rover camera dock: a plate that screws onto the OpenFrame Two top frame with a tower that
holds an Apple USB-C cable plug pointing up. The XIAO ESP32S3 Sense, in Seeed's official case,
plugs onto it USB-C end down (antenna up) and sits in a collar; lift it off to remove.

Run: cad/.venv/bin/python housing/housing.py   -> housing/out/cam-dock.stl
housing/check.py checks for collisions against the local chassis STEP.

World coordinates are the chassis STEP's: X across the car, Y along it (front = -Y),
Z up. All dimensions in mm. Edit the parameters below and re-run.
"""
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, export_stl

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
# Seeed official case, USB-C end (measured from Seeed's STL): 22.2 wide x ~6.4 deep for the
# first ~7 mm, then the camera side steps forward. USB-C centred across the width.
CASE_END = (22.2, 6.4)
CASE_SLIM_H = 7.0
# Apple USB-C cable plug: Apple's case guidance keeps cable plugs within 12.45 x 6.60 mm.
PLUG = (12.45, 6.60)

# ---- design parameters ----
PLATE_T = 3.0
PLATE_FRONT_EXTEND = 2.5               # overhang past the frame front: ~3 mm of material at the front screws
SCREW_ROWS = 2                         # front grid rows used (4 screws)
WALL = 2.0
CASE_CLEAR = 0.3                       # per side around the case
PLUG_CLEAR = 0.4                       # per side around the plug
COLLAR_DEPTH = 5.0                     # case drops this far into the collar (< CASE_SLIM_H)
PLUG_DEPTH = 24.0                      # plug channel below the collar; open at the back for the cable
TOWER_FRONT_Y = FRAME_FRONT_Y          # tower front flush with the frame front
TILT_UP_DEG = 0.0                      # tilt the whole tower back (camera up); 0 = level
SCREW_CLEAR_D = 2.9                    # M2.5 clearance
HEAD_D = 5.4                           # M2.5 button head, for clearance checks

PLATE_TOP = FRAME_TOP_Z + PLATE_T


def tower():
    """Tower in its own frame: origin at the bottom-front-centre, camera side toward -Y."""
    pocket = (CASE_END[0] + 2 * CASE_CLEAR, CASE_END[1] + 2 * CASE_CLEAR)
    channel = (PLUG[0] + 2 * PLUG_CLEAR, PLUG[1] + 2 * PLUG_CLEAR)
    depth = max(pocket[1], channel[1])
    outer = (pocket[0] + 2 * WALL, depth + 2 * WALL, PLUG_DEPTH + COLLAR_DEPTH)
    body = Pos(0, outer[1] / 2, outer[2] / 2) * Box(*outer)
    yc = WALL + depth / 2
    # Collar: the case's slim end sits here, on the ledges either side of the plug channel.
    body -= Pos(0, yc, PLUG_DEPTH + COLLAR_DEPTH / 2 + 0.5) * Box(pocket[0], pocket[1], COLLAR_DEPTH + 1)
    # Plug channel, full height, open at the back so the plug and cable slide in from behind
    # and the cable can bend out wherever the plug ends.
    body -= Pos(0, yc + outer[1] / 2, PLUG_DEPTH / 2 + 0.5) * Box(channel[0], channel[1] + outer[1], PLUG_DEPTH + 1.01)
    # Back of the collar open above the channel too, so the cable clears it.
    body -= Pos(0, yc + outer[1] / 2, PLUG_DEPTH + COLLAR_DEPTH / 2) * Box(channel[0], outer[1], COLLAR_DEPTH + 2)
    return body, outer


def build():
    t, outer = tower()
    t = t + Pos(0, outer[1] / 2, -4) * Box(outer[0], outer[1], 8)   # foot, trimmed below; keeps a tilted tower on the plate
    place = Pos(CX, TOWER_FRONT_Y, PLATE_TOP) * Rot(TILT_UP_DEG, 0, 0)
    t = place * t
    t = t - Pos(CX, TOWER_FRONT_Y, PLATE_TOP - 50) * Box(200, 400, 100)
    rows = GRID_Y[:SCREW_ROWS]
    plate_front = FRAME_FRONT_Y - PLATE_FRONT_EXTEND
    plate_len = rows[-1] + 5.5 - plate_front
    plate_w = GRID_X[1] - GRID_X[0] + 10
    plate = Pos(CX, plate_front + plate_len / 2, FRAME_TOP_Z + PLATE_T / 2) * Box(plate_w, plate_len, PLATE_T)
    dock = plate + t

    for (x0, x1), (y0, y1), depth in RELIEFS:
        dock -= Pos((x0 + x1) / 2, (y0 + y1) / 2, FRAME_TOP_Z + depth / 2 - 0.01) * Box(
            x1 - x0 + 2 * RELIEF_MARGIN, y1 - y0 + 2 * RELIEF_MARGIN, depth)

    assert all(abs(x - CX) > outer[0] / 2 + HEAD_D / 2 for x in GRID_X), 'screws would hit the tower'
    for y in rows:
        for x in GRID_X:
            dock -= Pos(x, y, FRAME_TOP_Z + PLATE_T / 2) * Cylinder(SCREW_CLEAR_D / 2, PLATE_T * 3)

    report = {
        'plate_mm': (round(plate_w, 1), round(plate_len, 1), PLATE_T),
        'tower_mm': tuple(round(v, 1) for v in outer),
        'collar_top_above_frame': round(PLATE_T + outer[2], 1),
        'screw_rows_y': [round(y, 1) for y in rows],
    }
    return dock, report


if __name__ == '__main__':
    dock, report = build()
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob('*.stl'):
        old.unlink()
    export_stl(dock, str(OUT / 'cam-dock.stl'))
    for k, v in report.items():
        print(f'{k}: {v}')
    bb = dock.bounding_box()
    print(f'cam dock print size: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, volume {dock.volume / 1000:.1f} cm3')
