"""Fit checks for housing.py (camera dock): no collisions with the chassis (needs the local chassis STEP)."""
import sys
from pathlib import Path

from build123d import import_step

sys.path.insert(0, str(Path(__file__).resolve().parent))
import housing as H

dock, report = H.build()

def overlap(a, b):
    try:
        return (a & b).volume
    except Exception:
        return float('nan')

chassis = import_step(str(H.CHASSIS_STEP))
parts = [s for s in chassis.solids() if s.bounding_box().max.X < 120 and s.bounding_box().max.Y < 20 and s.bounding_box().max.Z < 120]
hits = []
for s in parts:
    v = overlap(s, dock)
    if v > 0.5:
        b = s.bounding_box()
        hits.append(f'  dock hits chassis part {b.size.X:.0f}x{b.size.Y:.0f}x{b.size.Z:.0f} at z[{b.min.Z:.0f},{b.max.Z:.0f}] y[{b.min.Y:.0f},{b.max.Y:.0f}]: {v:.1f} mm3')
print('chassis collisions:', 'none' if not hits else '\n' + '\n'.join(hits))
