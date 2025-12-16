
from __future__ import annotations

import argparse
import sys
from pathlib import Path as _Path
sys.path.append(str(_Path(__file__).resolve().parents[1]))
import json
from pathlib import Path

from ba_star.scenarios import make_unified_scenario
from ba_star.ba_star import BAStarPlanner, BAStarConfig
from ba_star import viz


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", type=str, default="outputs")
    ap.add_argument("--max_steps", type=int, default=60000)
    ap.add_argument("--sense", type=str, default="N8", choices=["N8", "N4", "NONE"])
    ap.add_argument("--inflate", type=int, default=0)
    ap.add_argument("--prefer_cost", type=str, default="A_STAR", choices=["A_STAR", "EUCLIDEAN"])
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    scenario = make_unified_scenario()
    cfg = BAStarConfig(
        max_steps=args.max_steps,
        sense_mode=args.sense,
        inflate_obstacles=args.inflate,
        prefer_cost=args.prefer_cost,
        stop_if_no_candidates=True,
    )
    planner = BAStarPlanner(scenario.grid, scenario.start, scenario.start_theta, cfg=cfg)
    res = planner.run()

    # Figure 1: map
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    viz.plot_map(scenario.grid, ax=ax, title="Unified scenario map (ground truth)")
    viz.save_fig(str(outdir / "fig01_map.png"))

    # Figure 2: map with overlay cells (covered cells)
    fig, ax = plt.subplots(figsize=(7, 7))
    viz.plot_map(scenario.grid, ax=ax, title="Map with covered cells overlay")
    viz.overlay_cells(ax, res.covered_cells, marker_size=4.0, label="covered")
    ax.legend(loc="lower left", framealpha=0.85)
    viz.save_fig(str(outdir / "fig02_overlay_cells.png"))

    # Figure 3: full run  
    viz.plot_full_run(
        scenario.grid,
        covered=res.covered_cells,
        traj=res.trajectory_cells,
        out_path=str(outdir / "fig03_full_run.png"),
        title="BA* full run: covered cells and trajectory",
        start_point=scenario.start,
    )

    # If we have a backtracking event, visualize the first one
    if res.events:
        ev = res.events[0]
        viz.plot_candidates(
            scenario.grid,
            covered=res.covered_cells,
            candidates=ev.candidates,
            s_cp=ev.s_cp,
            s_sp=ev.s_sp,
            out_path=str(outdir / "fig04_candidates.png"),
        )
        viz.plot_astar_vs_smooth(
            scenario.grid,
            s_cp=ev.s_cp,
            s_sp=ev.s_sp,
            astar_path=ev.astar_path,
            smooth_path=ev.smooth_path,
            out_path=str(outdir / "fig05_astar_vs_smooth.png"),
        )

    summary = {
        "scenario": scenario.name,
        "start": list(scenario.start),
        "steps": res.steps,
        "coverage_rate": res.coverage_rate,
        "path_length": res.path_length,
        "num_backtrack_events": len(res.events),
        "first_event": None,
    }
    if res.events:
        ev = res.events[0]
        summary["first_event"] = {
            "s_cp": list(ev.s_cp),
            "s_sp": list(ev.s_sp) if ev.s_sp is not None else None,
            "num_candidates": len(ev.candidates),
            "astar_path_len": len(ev.astar_path),
            "smooth_path_len": len(ev.smooth_path),
        }

    (outdir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())