"""Fit checks and renders for housing.py (needs the local chassis STEP and Seeed XIAO model)."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from build123d import Box, Pos, import_step
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, str(Path(__file__).resolve().parent))
import housing as H

base, lid, _, xiao_loc, report = H.build()
xiao = xiao_loc * import_step(str(H.XIAO_STEP))
bw, bl, bt = H.BATTERY
battery = Pos(H.CX, report['battery_front_y'] + 2 + bl / 2, H.PLATE_TOP + bt / 2) * Box(bw, bl, bt)

def overlap(a, b):
    try:
        return (a & b).volume
    except Exception:
        return float('nan')

print(f'XIAO vs base: {overlap(xiao, base):.2f} mm3   XIAO vs lid: {overlap(xiao, lid):.2f} mm3')
print(f'battery vs base: {overlap(battery, base):.2f} mm3')
print(f'lid vs base: {overlap(lid, base):.2f} mm3')

chassis = import_step(str(H.CHASSIS_STEP))
parts = [s for s in chassis.solids() if s.bounding_box().max.X < 120 and s.bounding_box().max.Y < 20 and s.bounding_box().max.Z < 120]
hits = []
for s in parts:
    for name, part in (('base', base), ('lid', lid)):
        v = overlap(s, part)
        if v > 0.5:
            b = s.bounding_box()
            hits.append(f'  {name} hits chassis part {b.size.X:.0f}x{b.size.Y:.0f}x{b.size.Z:.0f} at z[{b.min.Z:.0f},{b.max.Z:.0f}] y[{b.min.Y:.0f},{b.max.Y:.0f}]: {v:.1f} mm3')
print('chassis collisions:', 'none' if not hits else '\n' + '\n'.join(hits))
above = [s for s in parts if s.bounding_box().max.Z > H.FRAME_TOP_Z + 0.5]
print('chassis parts rising above the frame top:', [(round(s.bounding_box().max.Z, 1), round(s.bounding_box().center().Y)) for s in above])

def tris(shape, color):
    out = []
    for s in shape.solids():
        v, f = s.tessellate(0.4, 0.4)
        v = np.array([(p.X, p.Y, p.Z) for p in v])
        out += [(v[list(face)], color) for face in f]
    return out

def render(groups, views, path, box=None):
    t = [x for g in groups for x in g]
    pts = np.array([p for tri, _ in t for p in tri])
    lo, hi = (pts.min(0), pts.max(0)) if box is None else box
    fig = plt.figure(figsize=(7 * len(views), 7))
    for k, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(1, len(views), k + 1, projection='3d')
        ax.add_collection3d(Poly3DCollection([a for a, _ in t], facecolors=[c for _, c in t], edgecolor='none'))
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev, azim); ax.set_title(title); ax.set_axis_off()
    plt.tight_layout(); plt.savefig(path, dpi=80); plt.close()

housing_g = tris(base, (0.35, 0.55, 0.85, 1)) + tris(lid, (0.5, 0.7, 0.95, 1))
xiao_g, bat_g = tris(xiao, (0.15, 0.6, 0.25, 1)), tris(battery, (0.85, 0.85, 0.85, 1))
chassis_g = []
for s in parts:
    chassis_g += tris(s, (0.3, 0.3, 0.3, 1))

local = H.ROOT / 'cad/source/renders'; local.mkdir(parents=True, exist_ok=True)
render([chassis_g, housing_g, bat_g, xiao_g], [(25, -125, 'on chassis (front-left)'), (90, -90, 'top'), (0, 180, 'side')], local / 'housing-on-chassis.png')
pod_box = (np.array([H.CX - 30, H.FRAME_FRONT_Y - 10, H.FRAME_TOP_Z - 5]), np.array([H.CX + 30, H.FRAME_FRONT_Y + 40, H.FRAME_TOP_Z + 45]))
render([housing_g, xiao_g], [(10, -100, 'pod front'), (20, 30, 'pod rear/side')], H.ROOT / 'housing/out/pod-closeup.png', pod_box)
render([housing_g, bat_g, xiao_g], [(30, -125, 'housing v1'), (90, -90, 'top')], H.ROOT / 'housing/out/housing-v1.png')
print('renders written')
