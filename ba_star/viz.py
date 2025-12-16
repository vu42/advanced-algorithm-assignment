
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Set, Tuple
import numpy as np
import matplotlib.pyplot as plt

from .grid_map import GridMap, Coord


def _cell_centers(cells: List[Coord]) -> Tuple[np.ndarray, np.ndarray]:
    xs = np.array([c[0] + 0.5 for c in cells], dtype=float)
    ys = np.array([c[1] + 0.5 for c in cells], dtype=float)
    return xs, ys


def plot_map(grid: GridMap, ax: Optional[plt.Axes] = None, title: Optional[str] = None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(grid.occ, cmap="gray_r", origin="upper")  # obstacles appear dark
    ax.set_xlim(-0.5, grid.spec.width - 0.5)
    ax.set_ylim(grid.spec.height - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title)
    return ax


def overlay_cells(ax: plt.Axes, cells: Set[Coord], marker_size: float = 6.0, label: Optional[str] = None):
    if not cells:
        return
    xs = np.array([c[0] + 0.5 for c in cells], dtype=float)
    ys = np.array([c[1] + 0.5 for c in cells], dtype=float)
    ax.scatter(xs, ys, s=marker_size, alpha=0.35, label=label)


def plot_trajectory(ax: plt.Axes, traj: List[Coord], linewidth: float = 1.3, label: Optional[str] = None):
    if not traj:
        return
    xs, ys = _cell_centers(traj)
    ax.plot(xs, ys, linewidth=linewidth, label=label)


def highlight_point(ax: plt.Axes, c: Coord, marker: str = "o", size: float = 60.0, label: Optional[str] = None):
    ax.scatter([c[0] + 0.5], [c[1] + 0.5], s=size, marker=marker, label=label)


def save_fig(path: str):
    plt.tight_layout(pad=0.2)
    plt.savefig(path, dpi=220, bbox_inches="tight")
    plt.close()


def plot_candidates(grid: GridMap, covered: Set[Coord], candidates: List[Coord], s_cp: Coord, s_sp: Coord, out_path: str):
    fig, ax = plt.subplots(figsize=(7, 7))
    plot_map(grid, ax=ax, title="Candidates L and selected s_sp")
    overlay_cells(ax, covered, marker_size=4.0, label="covered")
    if candidates:
        overlay_cells(ax, set(candidates), marker_size=14.0, label="L candidates")
    highlight_point(ax, s_cp, marker="x", size=90.0, label="s_cp")
    highlight_point(ax, s_sp, marker="*", size=120.0, label="s_sp")
    ax.legend(loc="lower left", framealpha=0.85)
    save_fig(out_path)


def plot_astar_vs_smooth(grid: GridMap, s_cp: Coord, s_sp: Coord, astar_path: List[Coord], smooth_path: List[Coord], out_path: str):
    fig, ax = plt.subplots(figsize=(7, 7))
    plot_map(grid, ax=ax, title="Backtracking path: A* vs A*SPT smoothing")
    if astar_path:
        plot_trajectory(ax, astar_path, linewidth=2.0, label="A* path P")
    if smooth_path:
        plot_trajectory(ax, smooth_path, linewidth=2.6, label="smoothed path P_hat")
    highlight_point(ax, s_cp, marker="x", size=90.0, label="s_cp")
    highlight_point(ax, s_sp, marker="*", size=120.0, label="s_sp")
    ax.legend(loc="lower left", framealpha=0.85)
    save_fig(out_path)


def plot_full_run(grid: GridMap, covered: Set[Coord], traj: List[Coord], out_path: str, title: str = "BA* run"):
    fig, ax = plt.subplots(figsize=(7, 7))
    plot_map(grid, ax=ax, title=title)
    overlay_cells(ax, covered, marker_size=4.0, label="covered")
    plot_trajectory(ax, traj, linewidth=1.2, label="trajectory")
    ax.legend(loc="lower left", framealpha=0.85)
    save_fig(out_path)
