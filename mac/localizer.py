"""Where am I? Monte Carlo localization against the known apartment map (the idea of AMCL in Nav2).

Hundreds of guesses ("particles") of the rover's pose. Each move shifts every guess by what the
odometer says happened, plus noise sized like the real errors (distance ±15%, heading a few
degrees), so the cloud spreads where the rover might really be. At each stop the camera's free-floor
distances (or exact sim rays) across its view are compared with the distances the map predicts from
each guess; guesses that match keep their weight, the rest fade, and the cloud is resampled around
the good ones. The estimate is the weighted mean; the spread says how sure it is.

Map: rectangles in metres (sim/apartment/apartment.json walls + furniture; virtual doors are left out
because no sensor sees them). Poses: (x m, y m, heading deg).
"""
import math

import numpy as np

from route_planner import Grid

HFOV_DEG = 62.0
MAX_CM = 250.0           # beyond this the sensor says "clear"; compared as "far"
RAY_STEP_M = 0.025
HIT_M = 0.013            # a ray "hits" when its cell centre is this close to an obstacle
CELL_M = 0.025           # map grid (finer than the planner's: 5 cm cells read 5 cm short)
SIGMA_CM, SIGMA_FRAC = 10.0, 0.15
DIST_NOISE, TURN_NOISE_DEG, TURN_NOISE_FRAC = 0.15, 0.7, 0.03   # gyro heading is good; distance is not
CAM_FWD_M = 0.125        # the camera sits this far ahead of the rover's centre
BODY_CLEAR_M = 0.08
LOST_MATCH = -1.6        # mean log-likelihood per column below this: no guess explains the view      # a guess with the rover's centre this close to something is impossible


class Localizer:
    def __init__(self, rects, size, start, n=500, seed=0, spread=(0.05, 0.05, 3.0)):
        self.grid = Grid(rects, size, CELL_M)
        self.size = size
        self.rng = np.random.default_rng(seed)
        x, y, h = start
        self.p = np.column_stack([self.rng.normal(x, spread[0], n), self.rng.normal(y, spread[1], n),
                                  np.radians(self.rng.normal(h, spread[2], n))])
        self.w = np.full(n, 1.0 / n)
        self.match = 0.0

    # --- motion ---
    def predict(self, forward_m, left_m, turn_deg, bumped=False):
        """Move every guess by the odometer's step (in the rover's own frame) with matching noise.
        bumped: it hit something, so the wheels may have spun without moving it: anywhere from no
        movement to the full step."""
        n = len(self.w)
        dist = math.hypot(forward_m, left_m)
        if bumped:
            scale = self.rng.uniform(0.0, 1.1, n)
        else:
            scale = 1 + self.rng.normal(0, DIST_NOISE, n) if dist else np.ones(n)
        turn = math.radians(turn_deg) + np.radians(self.rng.normal(0, TURN_NOISE_DEG * min(1, dist / 0.25 + abs(turn_deg) / 10)
                                                                    + TURN_NOISE_FRAC * abs(turn_deg), n))
        h = self.p[:, 2]
        mid = h + turn / 2                  # move along the mean heading of the step
        self.p[:, 0] += scale * (forward_m * np.cos(mid) - left_m * np.sin(mid))
        self.p[:, 1] += scale * (forward_m * np.sin(mid) + left_m * np.cos(mid))
        self.p[:, 2] = (h + turn + math.pi) % (2 * math.pi) - math.pi

    # --- sensing ---
    def expected_cm(self, columns):
        """Forward distance (cm) each guess would see in each of `columns` directions across the view."""
        f = 0.5 / math.tan(math.radians(HFOV_DEG) / 2)
        bearings = -np.arctan2((np.arange(columns) + 0.5) / columns - 0.5, f)
        a = self.p[:, 2:3] + bearings[None, :]
        steps = np.arange(1, int(MAX_CM / 100 / RAY_STEP_M) + 1) * RAY_STEP_M
        cx = self.p[:, 0:1] + CAM_FWD_M * np.cos(self.p[:, 2:3])            # rays start at the camera
        cy = self.p[:, 1:2] + CAM_FWD_M * np.sin(self.p[:, 2:3])
        xs = cx[:, :, None] + np.cos(a)[:, :, None] * steps                  # particles x columns x steps
        ys = cy[:, :, None] + np.sin(a)[:, :, None] * steps
        i = np.clip((xs / CELL_M).astype(int), 0, self.grid.nx - 1)
        j = np.clip((ys / CELL_M).astype(int), 0, self.grid.ny - 1)
        hit = self.grid.dist[i, j] < HIT_M
        first = np.where(hit.any(axis=2), hit.argmax(axis=2), len(steps) - 1)
        rng_m = steps[first]
        return np.minimum(MAX_CM, rng_m * np.cos(bearings)[None, :] * 100)

    def update(self, observed_cm):
        """Weigh the guesses by how well their expected view matches the observed one; resample."""
        obs = np.minimum(MAX_CM, np.asarray(observed_cm, float))
        exp = self.expected_cm(len(obs))
        sigma = SIGMA_CM + SIGMA_FRAC * np.minimum(obs, exp)
        far = (obs >= MAX_CM - 1) & (exp >= MAX_CM - 1)
        z = np.where(far, 0.0, (obs - exp) / sigma)
        loglik = np.sum(np.log(0.9 * np.exp(-0.5 * z ** 2) + 0.1), axis=1)      # 10% outliers
        self.match = float(np.max(loglik) / len(obs))       # how well the best guess explains the view
        inside = np.array([self.grid.clearance(x, y) < BODY_CLEAR_M for x, y in self.p[:, :2]])
        loglik[inside] = -1e9
        logw = np.log(self.w + 1e-300) + loglik
        logw -= logw.max()
        self.w = np.exp(logw)
        self.w /= self.w.sum()
        if 1.0 / np.sum(self.w ** 2) < len(self.w) / 2:
            self.resample()
        if self.match < LOST_MATCH:                # nothing explains what it sees: it's lost nearby
            self.scatter()

    def scatter(self, share=0.3, reach=1.0):
        """Replace a share of the guesses with fresh ones spread around the estimate (recovery)."""
        (x, y, h), _ = self.estimate()
        n = len(self.w)
        k = int(n * share)
        idx = self.rng.choice(n, k, replace=False)
        self.p[idx, 0] = self.rng.normal(x, reach / 2, k)
        self.p[idx, 1] = self.rng.normal(y, reach / 2, k)
        self.p[idx, 2] = np.radians(h + self.rng.normal(0, 8, k))
        self.w[:] = 1.0 / n

    def resample(self):
        n = len(self.w)
        positions = (self.rng.random() + np.arange(n)) / n
        idx = np.searchsorted(np.cumsum(self.w), positions).clip(0, n - 1)
        self.p = self.p[idx]
        self.p[:, :2] += self.rng.normal(0, 0.01, (n, 2))                     # keep a little diversity
        self.p[:, 2] += np.radians(self.rng.normal(0, 0.5, n))
        self.w = np.full(n, 1.0 / n)

    # --- answer ---
    def estimate(self):
        """(x, y, heading deg), spread (m)."""
        w = self.w
        x, y = float(np.sum(w * self.p[:, 0])), float(np.sum(w * self.p[:, 1]))
        h = math.degrees(math.atan2(np.sum(w * np.sin(self.p[:, 2])), np.sum(w * np.cos(self.p[:, 2]))))
        spread = float(np.sqrt(np.sum(w * ((self.p[:, 0] - x) ** 2 + (self.p[:, 1] - y) ** 2))))
        return (x, y, h), spread
