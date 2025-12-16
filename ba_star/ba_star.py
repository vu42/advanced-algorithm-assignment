
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple
import math

import numpy as np

from .grid_map import GridMap, Coord
from .astar import astar, euclidean_heuristic, AStarResult
from .smoothing import astar_spt_smooth, SmoothResult


STATE_UNKNOWN = 0
STATE_COVERED = 1
STATE_OBSTACLE = 2


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # radians


@dataclass
class BAStarConfig:
    max_steps: int = 200000
    sense_mode: str = "N8"  # "N4" or "N8" or "NONE"
    inflate_obstacles: int = 0  # inflate ground-truth obstacles by this many cells
    prefer_cost: str = "A_STAR"  # "EUCLIDEAN" or "A_STAR"
    stop_if_no_candidates: bool = True


@dataclass
class BacktrackEvent:
    s_cp: Coord
    candidates: List[Coord]
    s_sp: Optional[Coord]
    astar_path: List[Coord]
    smooth_path: List[Coord]


@dataclass
class RunResult:
    trajectory_cells: List[Coord]
    covered_cells: Set[Coord]
    known_obstacles: Set[Coord]
    events: List[BacktrackEvent]
    steps: int
    coverage_rate: float
    path_length: float


def heading_to_dir(theta: float) -> Coord:
    """
    Map a continuous heading angle to the nearest cardinal direction as a delta (dx, dy).
    Conventions match the report's set (E=0, N=pi/2, W=pi, S=-pi/2), but here we use grid coords where y increases downward.
    Therefore:
    - North: dy = -1
    - South: dy = +1
    - East:  dx = +1
    - West:  dx = -1
    """
    # normalize to (-pi, pi]
    t = (theta + math.pi) % (2 * math.pi) - math.pi
    # nearest among [0, pi/2, pi, -pi/2]
    candidates = [
        (0.0, (1, 0)),              # East
        (math.pi/2, (0, -1)),       # North
        (math.pi, (-1, 0)),         # West
        (-math.pi/2, (0, 1)),       # South
    ]
    best = min(candidates, key=lambda c: abs((t - c[0] + math.pi) % (2*math.pi) - math.pi))
    return best[1]


class BAStarPlanner:
    def __init__(self, grid: GridMap, start_cell: Coord, start_theta: float = 0.0, cfg: Optional[BAStarConfig] = None):
        self.cfg = cfg or BAStarConfig()
        self.grid = grid.inflate_obstacles(self.cfg.inflate_obstacles)
        if not self.grid.is_free(start_cell):
            raise ValueError("start_cell must be free in the map")
        self.pose = Pose(float(start_cell[0]), float(start_cell[1]), float(start_theta))
        self.cell = start_cell

        # discovered map state: only store known non-unknown states; missing => unknown
        self.hatM: Dict[Coord, int] = {}
        self.covered: Set[Coord] = set()
        self.known_obs: Set[Coord] = set()
        self.trajectory: List[Coord] = [start_cell]

        self.events: List[BacktrackEvent] = []
        self.steps = 0

        self._sense()

        self._mark_covered(start_cell)

    def _get_state(self, c: Coord) -> int:
        return self.hatM.get(c, STATE_UNKNOWN)

    def _set_state(self, c: Coord, st: int) -> None:
        self.hatM[c] = st
        if st == STATE_OBSTACLE:
            self.known_obs.add(c)
        elif st == STATE_COVERED:
            self.covered.add(c)

    def _mark_covered(self, c: Coord) -> None:
        if self._get_state(c) == STATE_UNKNOWN:
            self._set_state(c, STATE_COVERED)

    def _sense(self) -> None:
        """
        Update discovered obstacles around current cell.
        """
        mode = self.cfg.sense_mode.upper()
        if mode == "NONE":
            return
        if mode == "N4":
            cells = [self.cell] + self.grid.neighbors4(self.cell)
        else:
            cells = [self.cell] + self.grid.neighbors8(self.cell)

        for c in cells:
            if self.grid.is_obstacle(c):
                self._set_state(c, STATE_OBSTACLE)

    def is_blocked(self, c: Coord) -> bool:
        # blocked if known covered or known obstacle or true obstacle (safety in simulation)
        if not self.grid.in_bounds(c):
            return True
        if self.grid.is_obstacle(c):
            return True
        st = self._get_state(c)
        return st in (STATE_COVERED, STATE_OBSTACLE)

    def is_uncovered_free(self, c: Coord) -> bool:
        """
        Free and uncovered relative to the discovered map.
        In simulation we also require ground-truth free.
        """
        return self.grid.is_free(c) and (self._get_state(c) == STATE_UNKNOWN)

    def _bm_next_cell(self, s: Coord) -> Optional[Coord]:
        """
        One BM step using the priority order N, S, E, W.
        Returns next cell, or None if all blocked.
        """
        # neighbors4 returns [N, S, E, W]
        for nb in self.grid.neighbors4(s):
            # BM moves into uncovered free tiles only
            if self.is_uncovered_free(nb):
                return nb
        return None

    def _is_critical(self, s: Coord) -> bool:
        # critical if all four directions blocked
        for nb in self.grid.neighbors4(s):
            if not self.is_blocked(nb):
                return False
        return True

    def _neighbors8_indexed(self, s: Coord) -> List[Coord]:
        """
        Return [s1..s8] in the report order.
        """
        nbs = self.grid.neighbors8(s)
        # grid.neighbors8 already returns the ordered list, but it filters by bounds.
        # We need all 8 positions, even out of bounds, to evaluate patterns consistently.
        x, y = s
        s1 = (x+1, y)
        s2 = (x+1, y-1)
        s3 = (x, y-1)
        s4 = (x-1, y-1)
        s5 = (x-1, y)
        s6 = (x-1, y+1)
        s7 = (x, y+1)
        s8 = (x+1, y+1)
        return [s1, s2, s3, s4, s5, s6, s7, s8]

    def _b_indicator(self, si: Coord, sj: Coord) -> int:
        """
        b(si, sj) = 1 if si is free and uncovered, and sj is blocked.
        """
        si_free = self.is_uncovered_free(si)
        sj_blocked = self.is_blocked(sj)
        return 1 if (si_free and sj_blocked) else 0

    def mu(self, s: Coord) -> int:
        """
        Corner detector from the report:
        mu(s)=b(s1,s8)+b(s1,s2)+b(s5,s6)+b(s5,s4)+b(s7,s6)+b(s7,s8)
        """
        s1, s2, s3, s4, s5, s6, s7, s8 = self._neighbors8_indexed(s)
        return (
            self._b_indicator(s1, s8)
            + self._b_indicator(s1, s2)
            + self._b_indicator(s5, s6)
            + self._b_indicator(s5, s4)
            + self._b_indicator(s7, s6)
            + self._b_indicator(s7, s8)
        )

    def build_candidates_L(self) -> List[Coord]:
        """
        Build L by scanning covered tiles and selecting those with mu(s) >= 1.
        """
        L: List[Coord] = []
        for s in self.covered:
            if self.mu(s) >= 1:
                L.append(s)
        return L

    def _passable_for_astar(self, c: Coord) -> bool:
        # backtracking moves only over covered tiles (including start and goal)
        if not self.grid.in_bounds(c):
            return False
        if self.grid.is_obstacle(c):
            return False
        return c in self.covered

    def _astar_path(self, start: Coord, goal: Coord) -> Optional[AStarResult]:
        return astar(
            start=start,
            goal=goal,
            neighbors_fn=lambda c: self.grid.neighbors4(c),
            passable_fn=self._passable_for_astar,
            heuristic_fn=euclidean_heuristic,
        )

    def select_start_point(self, s_cp: Coord, L: List[Coord]) -> Optional[Coord]:
        if not L:
            return None
        if self.cfg.prefer_cost.upper() == "EUCLIDEAN":
            return min(L, key=lambda s: euclidean_heuristic(s_cp, s))

        # A* cost over covered graph
        best_s = None
        best_cost = float("inf")
        for s in L:
            res = self._astar_path(s_cp, s)
            if res is None:
                continue
            if res.cost < best_cost:
                best_cost = res.cost
                best_s = s
        return best_s

    def _follow_cells(self, cells: List[Coord]) -> None:
        """
        Append the cell path to trajectory, updating sensing along the way.
        This is a discrete execution; continuous pose is not simulated beyond storing theta updates.
        """
        for c in cells[1:]:
            self.cell = c
            self.pose.x, self.pose.y = float(c[0]), float(c[1])
            self.trajectory.append(c)
            self.steps += 1
            self._sense()
            if self.steps >= self.cfg.max_steps:
                break

    def _heading_adjustment(self, s_sp: Coord) -> None:
        """
        Choose a discrete heading (N, E, S, W) in priority order that leads to an uncovered free neighbor.
        Set pose.theta accordingly.
        """
        # priority N, E, S, W
        candidates = [
            (math.pi/2, (0, -1)),   # North
            (0.0, (1, 0)),          # East
            (-math.pi/2, (0, 1)),   # South
            (math.pi, (-1, 0)),     # West
        ]
        x, y = s_sp
        for gamma, d in candidates:
            nb = (x + d[0], y + d[1])
            if self.is_uncovered_free(nb):
                self.pose.theta = gamma
                return
        # fallback: keep heading unchanged

    def run(self) -> RunResult:
        """
        Execute BA* until termination or max steps.
        """
        # Main loop: BM until critical, then backtrack.
        while self.steps < self.cfg.max_steps:
            # BM phase
            while self.steps < self.cfg.max_steps:
                nxt = self._bm_next_cell(self.cell)
                if nxt is None:
                    break
                self._follow_cells([self.cell, nxt])
                self._mark_covered(self.cell)

            s_cp = self.cell
            if not self._is_critical(s_cp):
                # could happen if BM stopped due to max steps
                break

            # build L and pick s_sp
            L = self.build_candidates_L()
            s_sp = self.select_start_point(s_cp, L)

            if s_sp is None:
                if self.cfg.stop_if_no_candidates:
                    break
                else:
                    # no candidates but continue? stop
                    break

            res = self._astar_path(s_cp, s_sp)
            if res is None:
                # if graph disconnected, remove this candidate and try again
                # in this simplified implementation, terminate
                break

            smooth = astar_spt_smooth(self.grid, res.path)

            # record event
            self.events.append(
                BacktrackEvent(
                    s_cp=s_cp,
                    candidates=sorted(L),
                    s_sp=s_sp,
                    astar_path=res.path,
                    smooth_path=smooth.path,
                )
            )

            # follow smoothed path (backtracking)
            self._follow_cells(smooth.path)

            # heading adjustment
            self._heading_adjustment(s_sp)

            # mark current (already covered)
            self._mark_covered(self.cell)

        # compute metrics
        free_total = int(np.sum(self.grid.occ == 0))
        covered_free = sum(1 for c in self.covered if self.grid.is_free(c))
        coverage_rate = (covered_free / free_total) if free_total > 0 else 0.0

        # path length in cells (each move is 1)
        path_length = max(0, len(self.trajectory) - 1) * self.grid.spec.tile_size

        return RunResult(
            trajectory_cells=self.trajectory,
            covered_cells=set(self.covered),
            known_obstacles=set(self.known_obs),
            events=list(self.events),
            steps=self.steps,
            coverage_rate=float(coverage_rate),
            path_length=float(path_length),
        )
