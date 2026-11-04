"""Route planning for a car-like rover that can't turn tighter than its measured radius.

Hybrid A*: searches over position and heading using the moves the rover can actually make: straight
STEP_M forward or back, and full-lock arcs of ARC_DEG forward or back at RADIUS_M (drive/motion.json).
The whole body (three circles along it) must stay CLEAR_M clear of walls, furniture and virtual
doors. Heuristic: obstacle-aware distance to the goal (a grid flood from the goal), so it doesn't
wander into dead ends. Reversing costs extra and changing direction costs extra, so it prefers
driving forward the way a person would.

The plan comes back as rover moves the drive server understands, consecutive pieces merged:
  [('forward', cm), ('left', deg), ('reverse_right', deg), ('back', cm), ...]

Rectangles: [{'x0', 'y0', 'x1', 'y1'}] in metres (sim/apartment/apartment.json), x east, y north.
Poses: (x m, y m, heading deg, 0 = east, + = left/counter-clockwise).
"""
import heapq
import math

import numpy as np

CELL_M = 0.05
HEADINGS = 32                       # 11.25° bins
STEP_M = 0.25
RADIUS_M = 1.325
ARC_DEG = 360 / HEADINGS            # one bin per arc piece: 26 cm along the arc
BODY = (-0.07, 0.0, 0.07)           # circle centres along the rover (m from its centre)
BODY_R = 0.095                      # covers the 25 x 17 cm footprint
CLEAR_M = 0.04                      # extra margin beyond the body
REVERSE_COST, SWITCH_COST, ARC_COST = 4.0, 0.6, 1.1     # reversing is blind on the real rover
GOAL_M = 0.25


class Grid:
    """Distance (m) from each cell centre to the nearest obstacle rectangle."""

    def __init__(self, rects, size):
        self.nx, self.ny = int(math.ceil(size[0] / CELL_M)), int(math.ceil(size[1] / CELL_M))
        xs = (np.arange(self.nx) + 0.5) * CELL_M
        ys = (np.arange(self.ny) + 0.5) * CELL_M
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        d = np.full(X.shape, 9.0)
        for r in rects:
            dx = np.maximum(np.maximum(r['x0'] - X, 0), X - r['x1'])
            dy = np.maximum(np.maximum(r['y0'] - Y, 0), Y - r['y1'])
            d = np.minimum(d, np.hypot(dx, dy))
        self.dist = d

    def clearance(self, x, y):
        i, j = int(x / CELL_M), int(y / CELL_M)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return 0.0
        return float(self.dist[i, j])

    def body_free(self, x, y, h):
        c, s = math.cos(h), math.sin(h)
        return all(self.clearance(x + b * c, y + b * s) > BODY_R + CLEAR_M for b in BODY)

    def flood(self, goal):
        """Obstacle-aware distance (m) from every cell to the goal, through cells the body fits in."""
        ok = self.dist > BODY_R + CLEAR_M - CELL_M
        far = np.full(self.dist.shape, np.inf)
        gi, gj = int(goal[0] / CELL_M), int(goal[1] / CELL_M)
        far[gi, gj] = 0.0
        heap = [(0.0, gi, gj)]
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while heap:
            d, i, j = heapq.heappop(heap)
            if d > far[i, j]:
                continue
            for di, dj, w in steps:
                a, b = i + di, j + dj
                if 0 <= a < self.nx and 0 <= b < self.ny and ok[a, b] and d + w * CELL_M < far[a, b]:
                    far[a, b] = d + w * CELL_M
                    heapq.heappush(heap, (far[a, b], a, b))
        return far


def primitives():
    """(name, direction, list of (dx, dy, dh) samples in the rover's frame, length m)."""
    out = []
    for direction in (1, -1):
        samples = [(direction * STEP_M * k / 5, 0.0, 0.0) for k in range(1, 6)]
        out.append(('forward' if direction > 0 else 'back', direction, samples, STEP_M))
        for side in (1, -1):        # side +1: heading turns left
            # forward with left steer turns left; reversing with the wheels steered right turns left too
            dth = math.radians(ARC_DEG) * side
            samples = []
            for k in range(1, 6):
                t = dth * k / 5
                # arc of radius R, centre to the side the rover turns toward when moving forward
                sx, sy = RADIUS_M * math.sin(abs(t)), RADIUS_M * (1 - math.cos(t)) * (1 if side > 0 else -1)
                if direction < 0:   # reversing: same heading change, mirrored behind (and to the other side)
                    sx, sy = -sx, -sy
                samples.append((sx, sy, t))
            name = ('left' if side > 0 else 'right') if direction > 0 else ('reverse_left' if side > 0 else 'reverse_right')
            out.append((name, direction, samples, RADIUS_M * math.radians(ARC_DEG)))
    return out


PRIMS = primitives()


def plan(rects, size, start, goal, max_expand=200000):
    """start (x, y, heading deg), goal (x, y). Returns (moves, poses) or (None, why)."""
    grid = Grid(rects, size)
    if not grid.body_free(start[0], start[1], math.radians(start[2])):
        return None, 'the start is inside or touching something'
    if grid.clearance(*goal) < BODY_R + CLEAR_M:
        return None, 'the goal is inside or too close to something'
    far = grid.flood(goal)
    h_of = lambda x, y: far[min(grid.nx - 1, int(x / CELL_M)), min(grid.ny - 1, int(y / CELL_M))]
    if not np.isfinite(h_of(start[0], start[1])):
        return None, 'no way through (walls, furniture or virtual doors in the way)'
    key = lambda x, y, h: (int(x / CELL_M), int(y / CELL_M), int(round(h / (2 * math.pi / HEADINGS))) % HEADINGS)
    s = (start[0], start[1], math.radians(start[2]))
    best = {key(*s): 0.0}
    parent = {}
    heap = [(h_of(s[0], s[1]), 0.0, s, None, 0)]
    n = 0
    while heap and n < max_expand:
        f, g, (x, y, h), prim, direction = heapq.heappop(heap)
        k = key(x, y, h)
        if g > best.get(k, math.inf) + 1e-9:
            continue
        n += 1
        if math.hypot(x - goal[0], y - goal[1]) <= GOAL_M:
            moves, poses = [], [(x, y, h)]
            while k in parent:
                k, name, pose = parent[k]
                moves.append(name)
                poses.append(pose)
            moves.reverse()
            poses.reverse()
            return merge(moves), [(px, py, math.degrees(ph)) for px, py, ph in poses]
        c, sn = math.cos(h), math.sin(h)
        for name, d, samples, length in PRIMS:
            ok = True
            for dx, dy, dh in samples:
                px, py = x + dx * c - dy * sn, y + dx * sn + dy * c
                if not grid.body_free(px, py, h + dh):
                    ok = False
                    break
            if not ok:
                continue
            dx, dy, dh = samples[-1]
            nxt = (x + dx * c - dy * sn, y + dx * sn + dy * c, (h + dh + math.pi) % (2 * math.pi) - math.pi)
            cost = length * (REVERSE_COST if d < 0 else 1.0) * (ARC_COST if dh else 1.0)
            if direction and d != direction:
                cost += SWITCH_COST
            ng = g + cost
            nk = key(*nxt)
            if ng < best.get(nk, math.inf):
                best[nk] = ng
                parent[nk] = (k, name, (x, y, h))
                heapq.heappush(heap, (ng + h_of(nxt[0], nxt[1]), ng, nxt, name, d))
    return None, f'no route found ({n} states searched)'


def merge(names):
    """Primitive names -> drive-server moves: consecutive pieces of the same kind become one move."""
    out = []
    for name in names:
        amount = STEP_M * 100 if name in ('forward', 'back') else ARC_DEG
        if out and out[-1][0] == name:
            out[-1] = (name, round(out[-1][1] + amount, 1))
        else:
            out.append((name, round(amount, 1)))
    return out
