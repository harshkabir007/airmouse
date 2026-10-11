"""Rescue-grid helper: map-frame coordinates <-> grid labels such as "C7".

Convention (matches the GCS view):
  * map +x points INTO the maze from the entry (screen up), map +y points left.
  * arena origin = the left-hand corner of the entry wall, seen when facing into the maze.
  * columns (letters A, B, ...) run left -> right, rows (1, 2, ...) run from the
    entry wall inward. So "A1" is the cell at the entry wall on your left.

Change `arena.*` parameters to match the organisers' grid on the day.
"""
import math
from dataclasses import asdict, dataclass

from airmouse.params import declare

ARENA_DEFAULTS = {
    'origin_x': 1.0,
    'origin_y': 7.5,
    'cell_size': 1.0,
    'cols': 15,
    'rows': 15,
}


def col_letters(i):
    """0 -> A, 25 -> Z, 26 -> AA."""
    s = ''
    i += 1
    while i > 0:
        i, r = divmod(i - 1, 26)
        s = chr(65 + r) + s
    return s


@dataclass
class ArenaGrid:
    origin_x: float
    origin_y: float
    cell_size: float
    cols: int
    rows: int

    def bounds(self):
        """(xmin, xmax, ymin, ymax) of the arena in the map frame."""
        return (self.origin_x, self.origin_x + self.rows * self.cell_size,
                self.origin_y - self.cols * self.cell_size, self.origin_y)

    def contains(self, x, y, margin=0.0):
        xmin, xmax, ymin, ymax = self.bounds()
        return xmin - margin <= x <= xmax + margin and ymin - margin <= y <= ymax + margin

    def cell_of(self, x, y):
        c = math.floor((self.origin_y - y) / self.cell_size)
        r = math.floor((x - self.origin_x) / self.cell_size)
        if 0 <= c < self.cols and 0 <= r < self.rows:
            return c, r
        return None

    def label(self, x, y):
        cr = self.cell_of(x, y)
        if cr is None:
            return 'OUT'
        return f'{col_letters(cr[0])}{cr[1] + 1}'

    def to_dict(self):
        return asdict(self)


def declare_arena(node):
    vals = {k: declare(node, f'arena.{k}', v) for k, v in ARENA_DEFAULTS.items()}
    return ArenaGrid(float(vals['origin_x']), float(vals['origin_y']), float(vals['cell_size']),
                     int(vals['cols']), int(vals['rows']))
