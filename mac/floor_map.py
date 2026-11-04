"""Top-down occupancy map from the camera's free-floor distances and the rover's pose.

Each camera column looks along a bearing (from the field of view); depth_service gives how far the
floor is free along it. From the pose (x, y in cm and heading, from the drive server's odometer:
distance from drive time, heading from the gyro stick), each column is a ray: cells along it are
seen free, the cell where an obstacle stands is seen occupied. Evidence adds up as log-odds, so
repeated views firm up the map and a stray reading fades.

x is the way the rover faced at the last odometer reset, y to its left (cm). Drifts with the pose:
fine for a room, smears over long drives until landmarks or LiDAR correct the position.
"""
import math

import numpy as np

CELL_CM = 5.0
SIZE_CM = 1400.0           # 14 m square, the reset position in the middle
HFOV_DEG = 62.0            # XIAO OV3660 lens, horizontal (approximate)
FREE, HIT = -0.4, 0.9      # log-odds per view
LIMIT = 4.0
RANGE_CM = 250.0           # trust free floor out to here; "clear to the horizon" stops here, no hit


class FloorMap:
    def __init__(self, cell_cm=CELL_CM, size_cm=SIZE_CM, hfov_deg=HFOV_DEG):
        self.cell, self.hfov = cell_cm, hfov_deg
        self.n = int(size_cm / cell_cm)
        self.reset()

    def reset(self):
        self.odds = np.zeros((self.n, self.n), np.float32)
        self.trail = []
        self.views = 0

    def cell_of(self, x_cm, y_cm):
        """Grid (row, col): rows grow toward -x... drawn with +x up, +y left."""
        c = self.n // 2
        return int(round(c - x_cm / self.cell)), int(round(c - y_cm / self.cell))

    def bearings(self, columns):
        """Bearing (radians, + = left) of each of `columns` equal-width image columns."""
        f = 0.5 / math.tan(math.radians(self.hfov) / 2)        # focal length in image widths
        u = (np.arange(columns) + 0.5) / columns - 0.5           # -0.5 (left edge) .. 0.5 (right edge)
        return -np.arctan2(u, f)

    def add_view(self, x_cm, y_cm, heading_deg, free_cm, max_cm):
        """free_cm: free floor distance per column, left to right; max_cm = 'clear to the horizon'."""
        h = math.radians(heading_deg)
        free_cell = np.zeros_like(self.odds, bool)
        hit_cell = np.zeros_like(self.odds, bool)
        for bearing, dist in zip(self.bearings(len(free_cm)), free_cm):
            hit = dist < min(max_cm, RANGE_CM)
            length = min(dist, RANGE_CM)
            a = h + bearing
            steps = max(1, int(length / (self.cell / 2)))
            for k in range(steps):
                d = length * k / steps
                r, c = self.cell_of(x_cm + d * math.cos(a), y_cm + d * math.sin(a))
                if 0 <= r < self.n and 0 <= c < self.n:
                    free_cell[r, c] = True
            if hit:
                r, c = self.cell_of(x_cm + length * math.cos(a), y_cm + length * math.sin(a))
                if 0 <= r < self.n and 0 <= c < self.n:
                    hit_cell[r, c] = True
        free_cell &= ~hit_cell
        self.odds[free_cell] += FREE
        self.odds[hit_cell] += HIT
        np.clip(self.odds, -LIMIT, LIMIT, out=self.odds)
        self.views += 1
        if not self.trail or math.hypot(x_cm - self.trail[-1][0], y_cm - self.trail[-1][1]) > 2:
            self.trail.append((x_cm, y_cm))

    def stats(self):
        return {'views': self.views, 'free_m2': round(float((self.odds < -1).sum()) * (self.cell / 100) ** 2, 2),
                'obstacle_cells': int((self.odds > 1).sum()), 'cell_cm': self.cell}

    def render(self, pose=None, scale=3, crop=True):
        """BGR image: unknown dark, free blue-grey, obstacles red; trail and rover."""
        import cv2
        img = np.zeros((self.n, self.n, 3), np.uint8)
        img[:] = (34, 24, 18)
        img[self.odds < -1] = (82, 58, 42)
        img[self.odds > 1] = (90, 90, 240)
        known = np.argwhere(np.abs(self.odds) > 1)
        r0 = c0 = 0
        if crop and len(known):
            pad = int(100 / self.cell)
            r0, c0 = max(0, known[:, 0].min() - pad), max(0, known[:, 1].min() - pad)
            r1, c1 = min(self.n, known[:, 0].max() + pad), min(self.n, known[:, 1].max() + pad)
            img = img[r0:r1, c0:c1]
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

        def px(x, y):
            r, c = self.cell_of(x, y)
            return (int((c - c0 + 0.5) * scale), int((r - r0 + 0.5) * scale))
        for (xa, ya), (xb, yb) in zip(self.trail, self.trail[1:]):
            cv2.line(img, px(xa, ya), px(xb, yb), (250, 165, 96), 1)
        if pose:
            x, y, hd = pose
            h = math.radians(hd)
            tip = px(x + 18 * math.cos(h), y + 18 * math.sin(h))
            left = px(x + 8 * math.cos(h + 2.5), y + 8 * math.sin(h + 2.5))
            right = px(x + 8 * math.cos(h - 2.5), y + 8 * math.sin(h - 2.5))
            cv2.fillPoly(img, [np.array([tip, left, right])], (11, 158, 245))
        bar = int(100 / self.cell * scale)                       # 1 m scale bar
        cv2.line(img, (8, img.shape[0] - 10), (8 + bar, img.shape[0] - 10), (220, 220, 220), 2)
        cv2.putText(img, '1 m', (12 + bar, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)
        return img
