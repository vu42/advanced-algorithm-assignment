
# BA* minimal simulator and figure generator

This is a small Python implementation of BA* for grid-based simulation.
It is designed to support an assignment report workflow:

- Keep algorithm code general and parameterized.
- Use a scenario config for a unified walkthrough and figure generation.
- Generate figures for the report (map, trajectory, candidates, A* vs smoothed path).

## Setup

Python 3.10+ recommended.

Install dependencies:

```bash
pip install numpy matplotlib pillow
```

## Run the unified scenario

From this folder:

```bash
python scripts/run.py --outdir outputs
```

Outputs:

- `outputs/fig01_map.png`
- `outputs/fig02_full_run.png`
- `outputs/fig03_candidates.png` (only if backtracking occurs)
- `outputs/fig04_astar_vs_smooth.png` (only if backtracking occurs)
- `outputs/summary.json`

## Notes

- The environment is a discrete occupancy grid.
- The robot moves between grid cells (tile centers).
- Covered tiles are treated as blocked for BM and for corner detection (consistent with the report's predicates).
- Backtracking uses A* on the graph induced by covered cells, with a greedy line-of-sight smoother (A*SPT-like).
- Line-of-sight is implemented using a Bresenham discretization over the grid cells.
