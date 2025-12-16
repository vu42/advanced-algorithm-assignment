
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Optional
from .grid_map import GridMap, Coord


@dataclass
class SmoothResult:
    path: List[Coord]
    removed: int


def astar_spt_smooth(grid: GridMap, path: List[Coord]) -> SmoothResult:
    """
    A simplified A*SPT-like smoothing:
    greedily jump to the farthest line-of-sight point along the A* path.
    """
    if not path:
        return SmoothResult(path=[], removed=0)
    if len(path) <= 2:
        return SmoothResult(path=path[:], removed=0)

    out: List[Coord] = [path[0]]
    i = 0
    removed = 0
    n = len(path)

    while i < n - 1:
        # choose farthest visible j
        j = n - 1
        while j > i + 1:
            if grid.line_of_sight(path[i], path[j]):
                break
            j -= 1
        # if nothing found, fall back to immediate neighbor
        if j == i:
            j = i + 1
        if j > i + 1:
            removed += (j - i - 1)
        out.append(path[j])
        i = j

    return SmoothResult(path=out, removed=removed)
