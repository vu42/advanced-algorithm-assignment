
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

Coord = Tuple[int, int]  # (x, y)


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    tile_size: float = 1.0  # physical size per cell, optional


class GridMap:
    """
    Binary occupancy grid map.

    Conventions:
    - occupancy[y, x] == 1 means obstacle
    - occupancy[y, x] == 0 means free
    - coordinates are integer cell indices (x, y), with 0 <= x < width, 0 <= y < height
    - y increases downward (array indexing convention)
    """
    def __init__(self, occupancy: np.ndarray, tile_size: float = 1.0):
        if occupancy.ndim != 2:
            raise ValueError("occupancy must be a 2D array")
        self.occ = (occupancy > 0).astype(np.uint8)
        self.h, self.w = self.occ.shape
        self.spec = GridSpec(width=self.w, height=self.h, tile_size=float(tile_size))

    @staticmethod
    def from_binary_image(path: str, obstacle_is_black: bool = True, threshold: int = 128, tile_size: float = 1.0) -> "GridMap":
        from PIL import Image
        img = Image.open(path).convert("L")
        arr = np.array(img)
        if obstacle_is_black:
            occ = (arr < threshold).astype(np.uint8)
        else:
            occ = (arr >= threshold).astype(np.uint8)
        return GridMap(occ, tile_size=tile_size)

    def in_bounds(self, c: Coord) -> bool:
        x, y = c
        return 0 <= x < self.w and 0 <= y < self.h

    def is_obstacle(self, c: Coord) -> bool:
        x, y = c
        return bool(self.occ[y, x])

    def is_free(self, c: Coord) -> bool:
        return self.in_bounds(c) and (not self.is_obstacle(c))

    def neighbors4(self, c: Coord) -> List[Coord]:
        """
        Order: N, S, E, W (consistent with the BA* priority rule used in the report),
        where N = (x, y-1) and S = (x, y+1).
        """
        x, y = c
        cand = [(x, y-1), (x, y+1), (x+1, y), (x-1, y)]
        return [p for p in cand if self.in_bounds(p)]

    def neighbors8(self, c: Coord) -> List[Coord]:
        """
        Order matches the report indexing:
        s1 E, s2 NE, s3 N, s4 NW, s5 W, s6 SW, s7 S, s8 SE.
        """
        x, y = c
        cand = [
            (x+1, y),     # s1 E
            (x+1, y-1),   # s2 NE
            (x,   y-1),   # s3 N
            (x-1, y-1),   # s4 NW
            (x-1, y),     # s5 W
            (x-1, y+1),   # s6 SW
            (x,   y+1),   # s7 S
            (x+1, y+1),   # s8 SE
        ]
        return [p for p in cand if self.in_bounds(p)]

    def inflate_obstacles(self, radius_cells: int) -> "GridMap":
        if radius_cells <= 0:
            return GridMap(self.occ.copy(), tile_size=self.spec.tile_size)

        ys, xs = np.where(self.occ == 1)
        out = self.occ.copy()
        for x, y in zip(xs, ys):
            x0 = max(0, x - radius_cells)
            x1 = min(self.w - 1, x + radius_cells)
            y0 = max(0, y - radius_cells)
            y1 = min(self.h - 1, y + radius_cells)
            out[y0:y1+1, x0:x1+1] = 1
        return GridMap(out, tile_size=self.spec.tile_size)

    def bresenham_line(self, a: Coord, b: Coord) -> List[Coord]:
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        pts: List[Coord] = []
        while True:
            pts.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return pts

    def line_of_sight(self, a: Coord, b: Coord) -> bool:
        for p in self.bresenham_line(a, b):
            if not self.is_free(p):
                return False
        return True
