"""Fit checks for housing.py (pod mount + lid) (needs the local chassis STEP and Seeed XIAO model)."""
import sys
from pathlib import Path

from build123d import Box, Pos, import_step

sys.path.insert(0, str(Path(__file__).resolve().parent))
import housing as H

base, lid, _, xiao_loc, report = H.build()   # base = pod mount
xiao = xiao_loc * import_step(str(H.XIAO_STEP))

def overlap(a, b):
    try:
        return (a & b).volume
    except Exception:
        return float('nan')

print(f'XIAO vs base: {overlap(xiao, base):.2f} mm3   XIAO vs lid: {overlap(xiao, lid):.2f} mm3')
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
