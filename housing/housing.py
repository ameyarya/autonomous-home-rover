"""Rover camera pod mount: XIAO ESP32S3 Sense pod on a small plate that screws onto the
OpenFrame Two top frame, plus a pod lid. The power bank is strapped to the chassis separately.

Run: cad/.venv/bin/python housing/housing.py   -> housing/out/pod-mount.stl, pod-lid.stl
housing/check.py checks fit and collisions against the local chassis STEP.

World coordinates are the chassis STEP's: X across the car, Y along it (front = -Y),
Z up. All dimensions in mm. Edit the parameters below and re-run.
"""
from pathlib import Path

from build123d import Box, Cylinder, Location, Pos, Rot, Solid, export_stl

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
# Steering gear that rises 0.43 mm above the frame top: pocket in the plate underside.
RELIEFS = [((-15.5, -10.5), (-147.3, -119.6), 1.5)]   # (x range, y range, depth)
RELIEF_MARGIN = 1.5

# ---- parts ----
XIAO_STACK = (24.4, 14.0, 17.8)        # Seeed model: length (USB-C end) x thickness (camera side) x width
LENS_IN_MODEL = (8.75, -6.75)          # lens centre (x, z) in the Seeed model frame
XIAO_MODEL_MIN = (-9.0, -0.2, -15.0)   # Seeed model bounding-box minimum

# ---- design parameters ----
PLATE_T = 3.0
PLATE_FRONT_EXTEND = 2.5               # overhang past the frame front: ~3 mm of material at the front screws
SCREW_ROWS = 2                         # front grid rows used (4 screws)
WALL = 1.8
CLEAR = 0.4                            # per side around the XIAO stack
POD_FLOOR = 3.0                        # solid pedestal under the XIAO
TILT_UP_DEG = 5.0                      # camera tilt above horizontal
SCREW_CLEAR_D = 2.9                    # M2.5 clearance
HEAD_D = 5.4                           # M2.5 button head, for clearance checks
LENS_HOLE_D = (9.0, 14.0)              # inner, outer diameter of the conical lens window
USB_OPENING = (14.0, 8.5)              # width x height, USB-C plug overmould
LID_T, LID_LIP = 1.6, 2.0
ROOF = (44.0, 26.0)                    # lid roof; the 39 x 19 mm antenna sticks on top

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
    pod = pod + Pos(0, pod_outer[1] / 2, -4) * Box(pod_outer[0], pod_outer[1], 8)
    place = Pos(CX, FRAME_FRONT_Y, PLATE_TOP) * Rot(-TILT_UP_DEG, 0, 0)
    pod_w, lid_w = place * pod, place * lid
    pod_w = pod_w - Pos(CX, FRAME_FRONT_Y, PLATE_TOP - 50) * Box(200, 400, 100)   # trim below plate top

    rows = GRID_Y[:SCREW_ROWS]
    plate_front = FRAME_FRONT_Y - PLATE_FRONT_EXTEND
    plate_len = rows[-1] + 5.5 - plate_front
    plate_w = GRID_X[1] - GRID_X[0] + 10
    plate = Pos(CX, plate_front + plate_len / 2, FRAME_TOP_Z + PLATE_T / 2) * Box(plate_w, plate_len, PLATE_T)
    mount = plate + pod_w

    for (x0, x1), (y0, y1), depth in RELIEFS:
        mount -= Pos((x0 + x1) / 2, (y0 + y1) / 2, FRAME_TOP_Z + depth / 2 - 0.01) * Box(
            x1 - x0 + 2 * RELIEF_MARGIN, y1 - y0 + 2 * RELIEF_MARGIN, depth)

    # Screws beside the pod (it is narrower than the 50 mm row spacing) and behind it.
    assert all(abs(x - CX) > pod_outer[0] / 2 + HEAD_D / 2 for x in GRID_X), 'screws would hit the pod'
    for y in rows:
        for x in GRID_X:
            mount -= Pos(x, y, FRAME_TOP_Z + PLATE_T / 2) * Cylinder(SCREW_CLEAR_D / 2, PLATE_T * 3)

    report = {
        'plate_mm': (round(plate_w, 1), round(plate_len, 1), PLATE_T),
        'pod_outer_mm': tuple(round(v, 1) for v in pod_outer),
        'screw_rows_y': [round(y, 1) for y in rows],
        'mount_rear_y': round(plate_front + plate_len, 1),
    }
    return mount, lid_w, lid, place * xiao_loc, report


if __name__ == '__main__':
    mount, lid_world, lid_print, xiao_loc, report = build()
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob('*.stl'):
        old.unlink()
    export_stl(mount, str(OUT / 'pod-mount.stl'))
    export_stl(Rot(180, 0, 0) * lid_print, str(OUT / 'pod-lid.stl'))   # printed roof-down
    for k, v in report.items():
        print(f'{k}: {v}')
    bb = mount.bounding_box()
    print(f'pod mount print size: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, volume {mount.volume / 1000:.1f} cm3')
