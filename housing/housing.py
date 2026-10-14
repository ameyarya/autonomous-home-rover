"""Rover top housing v1: base plate + battery cradle + XIAO camera pod, and a pod lid.

Run: cad/.venv/bin/python housing/housing.py
Writes housing/out/housing-base.stl and housing-lid.stl. housing/check.py checks fit and
collisions against the local chassis STEP.

World coordinates are the chassis STEP's: X across the car, Y along it (front = -Y),
Z up. All dimensions in mm. Edit the parameters below and re-run.
"""
from pathlib import Path

from build123d import (Axis, Box, Compound, Cylinder, Location, Pos, Rot, Solid, export_stl,
                       import_step)

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'housing/out'
CHASSIS_STEP = ROOT / 'OpenFrame One 4W Final.step'           # local only, never committed
XIAO_STEP = ROOT / 'cad/source/xiao/model/Seeed Studio XIAO-ESP32-S3-Sense.step'

# ---- chassis interface (measured from the OpenFrame Two STEP) ----
FRAME_TOP_Z = 40.4
FRAME_FRONT_Y = -172.1
GRID_X = (-28.7, 21.3)                 # two rows of 2.5 mm holes, 50 mm apart
GRID_Y = [-170.1 + 22.5 * i for i in range(9)]
CX = sum(GRID_X) / 2                   # centreline of the hole grid

# ---- parts ----
XIAO_STACK = (24.4, 14.0, 17.8)        # Seeed model: length (USB-C end) x thickness (camera side) x width
LENS_IN_MODEL = (8.75, -6.75)          # lens centre (x, z) in the Seeed model frame
XIAO_MODEL_MIN = (-9.0, -0.2, -15.0)   # Seeed model bounding-box minimum
BATTERY = (70.8, 122.2, 7.7)           # iPhone Air MagSafe Battery (ChargerLAB): width x length x thickness

# ---- design parameters ----
PLATE_T = 3.0
WALL = 1.8
CLEAR = 0.4                            # per side around the XIAO stack
BAT_CLEAR = 0.6                        # per side around the battery
CRADLE_WALL_H = 6.0
CABLE_GAP = 30.0                       # between pod back and battery front edge (USB-C plugs)
POD_FLOOR = 3.0                        # solid pedestal under the XIAO
TILT_UP_DEG = 5.0                      # camera tilt above horizontal
SCREW_CLEAR_D = 2.9                    # M2.5 clearance
HEAD_D, HEAD_DEPTH = 5.4, 1.7          # M2.5 button head counterbore under the battery
LENS_HOLE_D = (9.0, 14.0)              # inner, outer diameter of the conical lens window
USB_OPENING = (14.0, 8.5)              # width x height, USB-C plug overmould
LID_T, LID_LIP = 1.6, 2.0
ROOF = (44.0, 26.0)                    # lid roof; the 39 x 19 mm antenna sticks on top
STRAP_SLOT = (3.0, 22.0)
# Chassis parts that rise just above the frame top get a pocket in the plate underside.
RELIEFS = [((-15.5, -10.5), (-147.3, -119.6), 1.5)]   # (x range, y range, depth); front steering gear, +0.6 mm
RELIEF_MARGIN = 1.5

PLATE_TOP = FRAME_TOP_Z + PLATE_T


def pod_frame():
    """Pod geometry in its own level frame: origin at the front-bottom-centre of the pod,
    camera looking along -Y. Returns (pod_body, lid, xiao_location)."""
    sx, sy, sz = XIAO_STACK
    inner = (sx + 2 * CLEAR, sy + 2 * CLEAR, sz + 2 * CLEAR)
    outer = (inner[0] + 2 * WALL, inner[1] + 2 * WALL, inner[2] + WALL + POD_FLOOR)
    body = Pos(0, outer[1] / 2, outer[2] / 2) * Box(*outer)
    cavity_z0 = POD_FLOOR
    cavity = Pos(0, WALL + inner[1] / 2, cavity_z0 + inner[2] / 2 + WALL) * Box(inner[0], inner[1], inner[2] + 2 * WALL)
    body -= cavity

    # XIAO placement: rotate the Seeed model 180 deg about Z so the camera (+Y) faces -Y.
    # After rotation the model's x -> -x, y -> -y. Stack centre on x=0, front face on the cavity front.
    mx0, my0, mz0 = XIAO_MODEL_MIN
    model_cx = mx0 + sx / 2
    tx = model_cx                                   # -(-model_cx) recentres after the flip
    ty = WALL + CLEAR + (my0 + sy)                  # model max y (camera face) lands on cavity front
    tz = cavity_z0 + WALL + CLEAR - mz0
    xiao_loc = Pos(tx, ty, tz) * Rot(0, 0, 180)

    lens_x = -(LENS_IN_MODEL[0]) + tx
    lens_z = LENS_IN_MODEL[1] + tz
    cone = Solid.make_cone(LENS_HOLE_D[1] / 2, LENS_HOLE_D[0] / 2, WALL + 0.2)
    body -= Location((lens_x, -0.1, lens_z), (-90, 0, 0)) * cone

    # USB-C end: the Seeed model has USB-C at +x, which the flip puts at -x.
    usb_z = tz + (-10.6 - 1.6) / 2
    body -= Pos(-outer[0] / 2, WALL + inner[1] / 2, usb_z) * Box(WALL * 3, USB_OPENING[0], USB_OPENING[1])

    # Vents / mic ports on the +x wall.
    for k in range(3):
        body -= Pos(outer[0] / 2, WALL + 3 + 4 * k, cavity_z0 + WALL + inner[2] / 2) * Box(WALL * 3, 1.6, inner[2] * 0.6)

    # Lid: roof plate with a lip that drops into the cavity, a slot for the antenna cable.
    top = outer[2]
    roof = Pos(0, outer[1] / 2, top + LID_T / 2) * Box(ROOF[0], ROOF[1], LID_T)
    lip = Pos(0, WALL + inner[1] / 2, top - LID_LIP / 2) * Box(inner[0] - 0.4, inner[1] - 0.4, LID_LIP)
    lip -= Pos(0, WALL + inner[1] / 2, top - LID_LIP / 2) * Box(inner[0] - 0.4 - 2 * 1.2, inner[1] - 0.4 - 2 * 1.2, LID_LIP + 1)
    lid = roof + lip
    lid -= Pos(inner[0] / 2 - 4, WALL + inner[1] - 3, top + LID_T / 2) * Box(5, 2.5, LID_T * 4)       # antenna cable
    for k in range(4):
        lid -= Pos(-inner[0] / 2 + 4 + 4 * k, WALL + 3, top + LID_T / 2) * Box(1.6, 5, LID_T * 4)     # vents
    rim = Pos(0, outer[1] / 2, top + LID_T + 0.5) * Box(ROOF[0], ROOF[1], 1.0)
    rim -= Pos(0, outer[1] / 2, top + LID_T + 0.5) * Box(ROOF[0] - 2.4, ROOF[1] - 2.4, 2.0)
    lid += rim
    return body, lid, xiao_loc, outer


def build():
    pod, lid, xiao_loc, pod_outer = pod_frame()

    # Pedestal below the pod so the tilted pod still meets the plate.
    pedestal = Pos(0, pod_outer[1] / 2, -4) * Box(pod_outer[0], pod_outer[1], 8)
    pod = pod + pedestal

    place = Pos(CX, FRAME_FRONT_Y, PLATE_TOP) * Rot(-TILT_UP_DEG, 0, 0)
    pod_w, lid_w = place * pod, place * lid
    pod_w = pod_w - Pos(CX, FRAME_FRONT_Y, PLATE_TOP - 50) * Box(200, 400, 100)   # trim below plate top

    pod_back = FRAME_FRONT_Y + pod_outer[1] + 2
    bat_front = pod_back + CABLE_GAP
    bw, bl, bt = BATTERY
    cradle_in = (bw + 2 * BAT_CLEAR, bl + 2 * BAT_CLEAR)
    cradle_out = (cradle_in[0] + 2 * 2.0, cradle_in[1] + 2 * 2.0)
    plate_len = (bat_front + cradle_out[1]) - FRAME_FRONT_Y
    plate = Pos(CX, FRAME_FRONT_Y + plate_len / 2, FRAME_TOP_Z + PLATE_T / 2) * Box(cradle_out[0], plate_len, PLATE_T)

    cradle_cy = bat_front + cradle_out[1] / 2
    cradle = Pos(CX, cradle_cy, PLATE_TOP + CRADLE_WALL_H / 2) * Box(cradle_out[0], cradle_out[1], CRADLE_WALL_H)
    cradle -= Pos(CX, cradle_cy, PLATE_TOP + CRADLE_WALL_H / 2) * Box(cradle_in[0], cradle_in[1], CRADLE_WALL_H + 1)
    cradle -= Pos(CX, bat_front + 1, PLATE_TOP + CRADLE_WALL_H / 2 + 1) * Box(24, 6, CRADLE_WALL_H)  # USB-C opening, front edge
    cradle -= Pos(CX, bat_front + cradle_out[1] - 1, PLATE_TOP + CRADLE_WALL_H) * Box(30, 6, CRADLE_WALL_H)  # finger notch, rear

    base = plate + cradle + pod_w
    for (x0, x1), (y0, y1), depth in RELIEFS:
        base -= Pos((x0 + x1) / 2, (y0 + y1) / 2, FRAME_TOP_Z + depth / 2 - 0.01) * Box(
            x1 - x0 + 2 * RELIEF_MARGIN, y1 - y0 + 2 * RELIEF_MARGIN, depth)

    # Strap slots in the overhang beyond the frame rails, two pairs along the battery.
    for y in (bat_front + bl * 0.25, bat_front + bl * 0.75):
        for x in (CX - cradle_out[0] / 2 + 3.5, CX + cradle_out[0] / 2 - 3.5):
            base -= Pos(x, y, FRAME_TOP_Z + PLATE_T / 2) * Box(STRAP_SLOT[0], STRAP_SLOT[1], PLATE_T * 3)

    # Screws: one row behind the pod (accessible), and the grid rows under the battery (counterbored).
    plate_end = FRAME_FRONT_Y + plate_len
    ys = [y for y in GRID_Y if pod_back + 3 < y < plate_end - 3]
    chosen = [ys[0]] + ([ys[len(ys) // 2], ys[-1]] if len(ys) > 2 else ys[1:])
    for y in chosen:
        for x in GRID_X:
            base -= Pos(x, y, FRAME_TOP_Z + PLATE_T / 2) * Cylinder(SCREW_CLEAR_D / 2, PLATE_T * 3)
            if y > bat_front:
                base -= Pos(x, y, PLATE_TOP - HEAD_DEPTH / 2 + 0.01) * Cylinder(HEAD_D / 2, HEAD_DEPTH)
    report = {
        'plate_mm': (round(cradle_out[0], 1), round(plate_len, 1), PLATE_T),
        'pod_outer_mm': tuple(round(v, 1) for v in pod_outer),
        'screw_rows_y': [round(y, 1) for y in chosen],
        'battery_front_y': round(bat_front, 1),
        'plate_rear_y': round(plate_end, 1),
    }
    return base, lid_w, lid, place * xiao_loc, report


if __name__ == '__main__':
    base, lid_world, lid_print, xiao_loc, report = build()
    OUT.mkdir(exist_ok=True)
    export_stl(base, str(OUT / 'housing-base.stl'))
    # Lid printed roof-down: flip it.
    lid_flat = Rot(180, 0, 0) * lid_print
    export_stl(lid_flat, str(OUT / 'housing-lid.stl'))
    # Quick test-fit print: the pod on a short stub of plate (no screws needed).
    for k, v in report.items():
        print(f'{k}: {v}')
    bb = base.bounding_box()
    print(f'base print size: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, volume {base.volume / 1000:.1f} cm3')
