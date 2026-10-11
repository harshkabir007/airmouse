"""Grid planning: inflation, wavefront (BFS) distances, frontiers, path extraction."""
import math

import numpy as np
from scipy import ndimage

NB8 = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)
SQUARE = np.ones((3, 3), bool)


def disk(r):
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r


def downsample(grid, f):
    """OccupancyGrid values (ny, nx) -> (occ, free, unknown) boolean grids at 1/f resolution."""
    ny, nx = grid.shape[0] // f, grid.shape[1] // f
    g = grid[:ny * f, :nx * f].reshape(ny, f, nx, f)
    occ = (g >= 50).any(axis=(1, 3))
    free = ((g >= 0) & (g < 50)).any(axis=(1, 3)) & ~occ
    return occ, free, ~occ & ~free


class PlanGrid:
    """Cropped, down-sampled view of the occupancy map used for planning.

    Cells are (row, col) = (iy, ix)."""

    def __init__(self, msg, plan_res, inflation, crop):
        res = msg.info.resolution
        f = max(1, int(round(plan_res / res)))
        W, H = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(H, W)
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        xmin, xmax, ymin, ymax = crop
        ix0 = int(np.clip(math.floor((xmin - ox) / res), 0, W))
        ix1 = int(np.clip(math.ceil((xmax - ox) / res), 0, W))
        iy0 = int(np.clip(math.floor((ymin - oy) / res), 0, H))
        iy1 = int(np.clip(math.ceil((ymax - oy) / res), 0, H))
        self.occ, self.free, self.unknown = downsample(grid[iy0:iy1, ix0:ix1], f)
        self.res = res * f
        self.ox = ox + ix0 * res
        self.oy = oy + iy0 * res
        self.shape = self.occ.shape
        r = int(math.ceil(inflation / self.res))
        blocked = ndimage.binary_dilation(self.occ, structure=disk(r)) if r > 0 else self.occ
        self.trav = self.free & ~blocked
        # metres from each cell to the nearest occupied cell
        self.clearance = ndimage.distance_transform_edt(~self.occ) * self.res

    def clearance_at(self, x, y):
        r, c = self.cell(x, y)
        if 0 <= r < self.shape[0] and 0 <= c < self.shape[1]:
            return float(self.clearance[r, c])
        return 0.0

    def cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def world(self, cell):
        return self.ox + (cell[1] + 0.5) * self.res, self.oy + (cell[0] + 0.5) * self.res

    def centers(self):
        xs = self.ox + (np.arange(self.shape[1]) + 0.5) * self.res
        ys = self.oy + (np.arange(self.shape[0]) + 0.5) * self.res
        return xs, ys


def nearest_true(mask, cell, radius):
    h, w = mask.shape
    r0, c0 = cell
    y0, y1 = max(0, r0 - radius), min(h, r0 + radius + 1)
    x0, x1 = max(0, c0 - radius), min(w, c0 + radius + 1)
    if y0 >= y1 or x0 >= x1:
        return None
    ys, xs = np.nonzero(mask[y0:y1, x0:x1])
    if ys.size == 0:
        return None
    k = int(np.argmin((ys + y0 - r0) ** 2 + (xs + x0 - c0) ** 2))
    return int(ys[k] + y0), int(xs[k] + x0)


def wavefront(trav, start):
    """8-connected BFS step count from start over trav (-1 = unreachable)."""
    dist = np.full(trav.shape, -1, np.int32)
    if not trav[start]:
        return dist
    front = np.zeros(trav.shape, bool)
    front[start] = True
    visited = front.copy()
    dist[start] = 0
    d = 0
    while True:
        d += 1
        nxt = ndimage.binary_dilation(front, structure=SQUARE) & trav & ~visited
        if not nxt.any():
            return dist
        dist[nxt] = d
        visited |= nxt
        front = nxt


def descend(dist, goal):
    """Follow decreasing distance from goal back to the start; returns start..goal."""
    if dist[goal] < 0:
        return None
    h, w = dist.shape
    path = [goal]
    cur = goal
    while dist[cur] > 0:
        best = None
        for dy, dx in NB8:
            n = (cur[0] + dy, cur[1] + dx)
            if 0 <= n[0] < h and 0 <= n[1] < w and 0 <= dist[n] < dist[cur]:
                if best is None or dist[n] < dist[best]:
                    best = n
        if best is None:
            return None
        cur = best
        path.append(cur)
    path.reverse()
    return path


def line_of_sight(trav, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    ys = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    xs = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(trav[ys, xs].all())


def simplify(path, trav, max_cells):
    """Greedy shortcutting with a cap on segment length (keeps waypoints close)."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = min(len(path) - 1, i + max_cells)
        while j > i + 1 and not line_of_sight(trav, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def frontier_mask(free, unknown, occ, wall_gap=2):
    """Free cells bordering unknown space, excluding cells within `wall_gap` cells of a wall.

    The exclusion drops fake frontiers along wall faces (unseen wall interiors, door jambs)
    while keeping real ones, e.g. the open end of a corridor."""
    near_wall = ndimage.binary_dilation(occ, structure=SQUARE, iterations=wall_gap)
    return free & ndimage.binary_dilation(unknown, structure=CROSS) & ~near_wall
