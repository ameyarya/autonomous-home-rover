"""Build the apartment for the simulator from the floor plan (floorplan.png).

Walls are the plan's thick black lines: dark pixels that survive a 4 x 4 opening (text, door arcs
and fixtures are thinner), merged into axis-aligned boxes. The top exterior wall (windows and the
balcony slider, drawn as thin double lines) is added by hand: the slider is its own geom with the
blinds' look, since the real rover drove into those blinds (2026-11-04). The patio is left out.

Scale: the bedroom is drawn as a 12' x 12' square, 157 px across -> 13.1 px per foot.
Furniture: furniture.json (from photos; boxes, cylinders, rugs, the bedroom carpet, and virtual
doors: no-go lines the simulated rover can't cross and its exact-depth rays treat as walls, while
the rendered camera doesn't see them), and floor
textures cut from the photos (tex_wood.png, tex_carpet.png) and a slatted tex_blinds.png, when
present. All of these are personal and stay local (gitignored).
Output: apartment.xml (worldbody geoms) and apartment_assets.xml (textures, materials) included by
the scene, apartment.json (wall and furniture rectangles in metres for collision checks). x to the right on the plan, y up the plan (north),
origin at the plan's bottom-left corner.

Run: mac/.vision-venv/bin/python sim/apartment/build_apartment.py   (numpy + opencv)
"""
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PX_PER_FT = 157 / 12.0
M_PER_PX = 0.3048 / PX_PER_FT
WALL_H = 2.4
MIN_RUN = 8                      # px: shorter runs are bits of text or symbols
TOP_Y = 93                       # plan row of the top exterior wall (thin double line)
SLIDER = (185, 300)              # plan columns of the balcony slider in that wall


def wall_mask(path):
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    dark = (img < 90).astype(np.uint8)
    walls = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((4, 4), np.uint8))
    walls[:TOP_Y - 2] = 0        # the patio and its storage room: outside
    return img.shape, walls


def boxes(mask):
    """Greedy cover of the mask by horizontal and vertical runs merged across rows/columns."""
    out = []
    for axis in (0, 1):          # 0: horizontal runs (rows), 1: vertical runs (columns)
        m = mask if axis == 0 else mask.T
        open_runs = {}           # (start, end) -> first line index
        for i in range(m.shape[0] + 1):
            row = m[i] if i < m.shape[0] else np.zeros(m.shape[1], np.uint8)
            runs, start = set(), None
            for j, v in enumerate(np.append(row, 0)):
                if v and start is None:
                    start = j
                elif not v and start is not None:
                    if j - start >= MIN_RUN:
                        runs.add((start, j))
                    start = None
            for run in list(open_runs):
                if run not in runs:
                    first = open_runs.pop(run)
                    if axis == 0:
                        out.append((run[0], first, run[1], i))         # x0, y0, x1, y1 (px)
                    else:
                        out.append((first, run[0], i, run[1]))
            for run in runs:
                open_runs.setdefault(run, i)
    # drop boxes inside another one
    out = [b for b in out if not any(o != b and o[0] <= b[0] and o[1] <= b[1] and o[2] >= b[2] and o[3] >= b[3]
                                     for o in out)]
    return out


def main():
    (h, w), mask = wall_mask(HERE / 'floorplan.png')
    ys, xs = np.nonzero(mask)
    x_min, y_max = xs.min(), ys.max()

    def metres(x, y):
        return (x - x_min) * M_PER_PX, (y_max - y) * M_PER_PX

    rects = []
    for x0, y0, x1, y1 in boxes(mask):
        (ax, ay), (bx, by) = metres(x0, y1), metres(x1, y0)
        rects.append({'name': f'wall{len(rects)}', 'x0': ax, 'y0': ay, 'x1': bx, 'y1': by, 'kind': 'wall'})
    right = xs.max()
    for name, (c0, c1), kind in (('top_west', (x_min, SLIDER[0]), 'wall'), ('slider', SLIDER, 'blinds'),
                                 ('top_east', (SLIDER[1], right), 'wall')):
        (ax, ay), (bx, by) = metres(c0, TOP_Y + 3), metres(c1, TOP_Y - 2)
        rects.append({'name': name, 'x0': ax, 'y0': ay, 'x1': bx, 'y1': by, 'kind': kind})
    size = metres(right, ys.min())
    top = metres(x_min, TOP_Y)[1]
    t = 0.1                       # outside walls, closed all round (the plan draws windows thin; the
    for name, x0, y0, x1, y1 in (('outer_w', -t, 0, 0, top), ('outer_e', size[0], 0, size[0] + t, top),   # entry
                                 ('outer_s', -t, -t, size[0] + t, 0)):   # door stays shut in the sim)
        rects.append({'name': name, 'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1, 'kind': 'wall'})
    geoms = []
    for r in rects:
        cx, cy = (r['x0'] + r['x1']) / 2, (r['y0'] + r['y1']) / 2
        sx, sy = max(0.01, (r['x1'] - r['x0']) / 2), max(0.01, (r['y1'] - r['y0']) / 2)
        mat = 'blinds' if r['kind'] == 'blinds' else 'wall'
        geoms.append(f'    <geom name="{r["name"]}" type="box" size="{sx:.3f} {sy:.3f} {WALL_H / 2:.2f}" '
                     f'pos="{cx:.3f} {cy:.3f} {WALL_H / 2:.2f}" material="{mat}"/>')
    furniture = json.loads((HERE / 'furniture.json').read_text()) if (HERE / 'furniture.json').exists() else {}
    tints = []                    # furniture colours, applied over a fine grain (the depth model needs texture)
    for b in furniture.get('boxes', []):
        rects.append({k: b[k] for k in ('name', 'x0', 'y0', 'x1', 'y1')} | {'kind': 'furniture'})
        geoms.append(f'    <geom name="{b["name"]}" type="box" size="{(b["x1"] - b["x0"]) / 2:.3f} '
                     f'{(b["y1"] - b["y0"]) / 2:.3f} {b["h"] / 2:.3f}" pos="{(b["x0"] + b["x1"]) / 2:.3f} '
                     f'{(b["y0"] + b["y1"]) / 2:.3f} {b["h"] / 2:.3f}" material="m_{b["name"]}"/>')
        tints.append((b['name'], b['rgb']))
    for c in furniture.get('cylinders', []):
        rects.append({'name': c['name'], 'x0': c['x'] - c['r'], 'y0': c['y'] - c['r'], 'x1': c['x'] + c['r'],
                      'y1': c['y'] + c['r'], 'kind': 'furniture'})
        geoms.append(f'    <geom name="{c["name"]}" type="cylinder" size="{c["r"]} {c["h"] / 2:.3f}" '
                     f'pos="{c["x"]} {c["y"]} {c["h"] / 2:.3f}" material="m_{c["name"]}"/>')
        tints.append((c['name'], c['rgb']))
    for b in furniture.get('barriers', []):         # virtual doors: no-go lines, not real objects
        rects.append({k: b[k] for k in ('name', 'x0', 'y0', 'x1', 'y1')} | {'kind': 'barrier'})
        geoms.append(f'    <geom name="{b["name"]}" type="box" size="{(b["x1"] - b["x0"]) / 2:.3f} '
                     f'{(b["y1"] - b["y0"]) / 2:.3f} .5" pos="{(b["x0"] + b["x1"]) / 2:.3f} '
                     f'{(b["y0"] + b["y1"]) / 2:.3f} .5" group="3" rgba="1 0 0 0" contype="0" conaffinity="0"/>')
    flat = [(r, 'rug', 'material="rug"') for r in furniture.get('rugs', [])]
    flat += [(r, 'carpet', 'material="carpet"') for r in furniture.get('carpet', [])]
    for i, (r, kind, look) in enumerate(flat):      # drawn on the floor, no collision
        geoms.append(f'    <geom name="{kind}{i}" type="box" size="{(r["x1"] - r["x0"]) / 2:.3f} '
                     f'{(r["y1"] - r["y0"]) / 2:.3f} .002" pos="{(r["x0"] + r["x1"]) / 2:.3f} '
                     f'{(r["y0"] + r["y1"]) / 2:.3f} .002" {look} contype="0" conaffinity="0"/>')
    xml = ('<!-- generated by build_apartment.py from floorplan.png; do not edit -->\n<mujocoinclude>\n'
           f'    <geom name="floor" type="plane" size="{size[0]:.2f} {size[1]:.2f} .1" '
           f'pos="{size[0] / 2:.2f} {size[1] / 2:.2f} 0" material="floor"/>\n'
           + '\n'.join(geoms) + '\n</mujocoinclude>\n')
    textures = []
    for name, png, fallback in (('floor', 'tex_wood.png', 'rgb1=".66 .58 .48" rgb2=".62 .54 .44" builtin="checker"'),
                                ('rug', 'tex_rug.png', 'rgb1=".75 .75 .80" rgb2=".55 .60 .70" builtin="checker"'),
                                ('grain', 'tex_grain.png', 'rgb1=".9 .9 .9" rgb2=".85 .85 .85" builtin="checker"'),
                                ('carpet', 'tex_carpet.png', 'rgb1=".72 .68 .62" rgb2=".70 .66 .60" builtin="flat"'),
                                ('blinds', 'tex_blinds.png', 'rgb1=".93 .94 .95" rgb2=".80 .82 .84" builtin="checker"')):
        src = f'file="{png}"' if (HERE / png).exists() else fallback + ' width="64" height="64"'
        repeat = {'floor': f'{size[0] / 0.8:.0f} {size[1] / 0.8:.0f}', 'carpet': '6 6', 'blinds': '8 1', 'rug': '2 2',
                  'grain': '3 3'}[name]
        textures.append(f'    <texture name="{name}" type="2d" {src}/>\n'
                        f'    <material name="{name}" texture="{name}" texrepeat="{repeat}"/>')
    textures.append('    <material name="wall" texture="grain" texrepeat="4 2" rgba="1 .98 .94 1"/>')
    for name, rgb in tints:
        textures.append(f'    <material name="m_{name}" texture="grain" texrepeat="2 2" '
                        f'rgba="{min(1, rgb[0] * 1.1):.2f} {min(1, rgb[1] * 1.1):.2f} {min(1, rgb[2] * 1.1):.2f} 1"/>')
    (HERE / 'apartment_assets.xml').write_text('<mujocoinclude>\n<asset>\n' + '\n'.join(textures)
                                               + '\n</asset>\n</mujocoinclude>\n')
    (HERE / 'apartment.xml').write_text(xml)
    plan = {'left': int(x_min), 'top': float(y_max - size[1] / M_PER_PX), 'width': w, 'height': h, 'm_per_px': M_PER_PX}
    (HERE / 'apartment.json').write_text(json.dumps({'size_m': size, 'm_per_px': M_PER_PX, 'walls': rects,
                                                     'floorplan': plan}, indent=1))
    print(f'{len(rects)} wall boxes, apartment {size[0]:.2f} x {size[1]:.2f} m')


if __name__ == '__main__':
    main()
