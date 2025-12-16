
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple, Optional, List
import numpy as np
import random

from .grid_map import GridMap, Coord


@dataclass
class Scenario:
    name: str
    grid: GridMap
    start: Coord
    start_theta: float = 0.0


def _add_border_walls(occ: np.ndarray) -> None:
    occ[0, :] = 1
    occ[-1, :] = 1
    occ[:, 0] = 1
    occ[:, -1] = 1


def make_unified_scenario(width: int = 90, height: int = 90) -> Scenario:
    """
    A hand-crafted map intended for debugging and figure generation.
    It is not guaranteed to reproduce specific numeric coordinates from the report;
    instead, it provides a stable environment where BA* triggers backtracking.
    """
    occ = np.zeros((height, width), dtype=np.uint8)
    _add_border_walls(occ)

    # Main room boundary inside
    # Create a large open rectangle region
    # Add a vertical obstacle wall with a narrow doorway to create two regions.
    x_wall = width // 2 + 6
    occ[10:height-10, x_wall] = 1
    # doorway
    door_y0 = height // 2 - 4
    occ[door_y0:door_y0+8, x_wall] = 0

    # Add an internal block in the right region to create corners
    occ[25:40, x_wall+8:x_wall+18] = 1

    # Add a thin horizontal wall in the left region with a gap
    y_wall = height // 2 + 10
    occ[y_wall, 8:x_wall-2] = 1
    occ[y_wall, 20:24] = 0  # gap

    grid = GridMap(occ)
    start = (12, height - 15)  # near bottom-left
    return Scenario(name="unified", grid=grid, start=start, start_theta=0.0)


def make_random_scenario(width: int = 90, height: int = 90, obstacle_prob: float = 0.18, seed: Optional[int] = None) -> Scenario:
    rng = random.Random(seed)
    occ = np.zeros((height, width), dtype=np.uint8)
    _add_border_walls(occ)
    for y in range(1, height-1):
        for x in range(1, width-1):
            if rng.random() < obstacle_prob:
                occ[y, x] = 1

    # Choose a random free start
    free = np.argwhere(occ == 0)
    if len(free) == 0:
        raise ValueError("no free cells in generated map")
    yx = free[rng.randrange(len(free))]
    start = (int(yx[1]), int(yx[0]))
    grid = GridMap(occ)
    return Scenario(name=f"random_seed_{seed}", grid=grid, start=start, start_theta=0.0)
