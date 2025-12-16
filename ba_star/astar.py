
from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple, Callable
import math

Coord = Tuple[int, int]


@dataclass
class AStarResult:
    path: List[Coord]
    cost: float
    expanded: int


def astar(
    start: Coord,
    goal: Coord,
    neighbors_fn: Callable[[Coord], List[Coord]],
    passable_fn: Callable[[Coord], bool],
    heuristic_fn: Callable[[Coord, Coord], float],
) -> Optional[AStarResult]:
    """
    A* on an implicit graph.
    """
    if start == goal:
        return AStarResult(path=[start], cost=0.0, expanded=0)

    open_heap: List[Tuple[float, int, Coord]] = []
    counter = 0

    g: Dict[Coord, float] = {start: 0.0}
    came_from: Dict[Coord, Coord] = {}

    f0 = heuristic_fn(start, goal)
    heapq.heappush(open_heap, (f0, counter, start))

    closed: Set[Coord] = set()
    expanded = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1

        if current == goal:
            # reconstruct
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return AStarResult(path=path, cost=g[current], expanded=expanded)

        for nb in neighbors_fn(current):
            if not passable_fn(nb):
                continue
            tentative = g[current] + 1.0
            if nb not in g or tentative < g[nb]:
                g[nb] = tentative
                came_from[nb] = current
                counter += 1
                f = tentative + heuristic_fn(nb, goal)
                heapq.heappush(open_heap, (f, counter, nb))

    return None


def euclidean_heuristic(a: Coord, b: Coord) -> float:
    return math.hypot(b[0]-a[0], b[1]-a[1])
