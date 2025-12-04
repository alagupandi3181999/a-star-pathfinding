#!/usr/bin/env python3
"""benchmark_runner.py

Ready-to-run benchmark script that imports your astar_dynamic module and
prints clean, human-readable benchmark results. Designed to be placed in the
same folder as astar_dynamic.py and run with Python 3.

Usage examples:
  python benchmark_runner.py               # runs default experiments
  python benchmark_runner.py --rows 100 100 --trials 5

Outputs:
- Prints a summary table to stdout
- Writes CSV file `benchmarks_runner_output.csv` in the current directory
"""

import argparse
import csv
import sys
import time
from typing import Callable, Tuple

# Import the module you implemented. It must be in the same directory or on PYTHONPATH.
import astar_dynamic as ad

# Define heuristics available to the runner
HEURISTICS = {
    "octile_admissible": (lambda a, b: ad.octile(a, b), 1.0),
    "weighted_manhattan_nonadmissible": (lambda a, b: ad.weighted_manhattan(a, b, w=1.5), 1.0),
}


def run_single(rows: int, cols: int, obs: float, heur_name: str, heur_func: Callable, heur_w: float, trials: int, seed: int):
    """Run `trials` independent runs and return aggregated results."""
    rng_master = ad.random.Random(seed)
    times = []
    expands = []
    costs = []
    found_count = 0

    for t in range(trials):
        seed_run = rng_master.randint(0, 2**31 - 1)
        rng = ad.random.Random(seed_run)
        g = ad.create_random_grid(rows, cols, obs, allow_diagonal=True, rng=rng)
        start = (0, 0)
        goal = (rows - 1, cols - 1)
        g.set_block(start[0], start[1], False)
        g.set_block(goal[0], goal[1], False)
        start_id = g.coord_to_id(*start)
        goal_id = g.coord_to_id(*goal)
        engine = ad.AStarEngine(g)

        res = engine.search(start_id, goal_id, heuristic_func=heur_func, heuristic_weight=heur_w)
        times.append(res.get("time_s", 0.0))
        expands.append(res.get("expanded", 0))
        cost = res.get("cost", ad.INF)
        costs.append(cost if cost < ad.INF else None)
        if cost < ad.INF:
            found_count += 1

    avg_time = sum(times) / len(times)
    avg_expanded = sum(expands) / len(expands)
    valid_costs = [c for c in costs if c is not None]
    avg_cost = sum(valid_costs) / len(valid_costs) if valid_costs else None

    return {
        "rows": rows,
        "cols": cols,
        "cells": rows * cols,
        "obs": obs,
        "heuristic": heur_name,
        "trials": trials,
        "found_fraction": f"{found_count}/{trials}",
        "avg_time_s": avg_time,
        "avg_expanded": avg_expanded,
        "avg_cost": avg_cost,
    }


def print_table(results):
    # print a clean table to stdout
    headers = ["size", "heuristic", "trials", "found", "avg_time_s", "avg_expanded", "avg_cost"]
    print("\nBenchmark summary:\n")
    row_fmt = "{size:>8}  {heuristic:30}  {trials:>6}  {found:>7}  {time:>10.6f}  {exp:>10.1f}  {cost:>10}"
    print("size     heuristic                       trials    found     avg_time   avg_exp    avg_cost")
    print("" + "-" * 96)
    for r in results:
        size = f"{r['rows']}x{r['cols']}"
        cost = f"{r['avg_cost']:.6f}" if r['avg_cost'] is not None else "None"
        print(row_fmt.format(size=size, heuristic=r['heuristic'], trials=r['trials'], found=r['found_fraction'], time=r['avg_time_s'], exp=r['avg_expanded'], cost=cost))
    print()


def write_csv(results, path="benchmarks_runner_output.csv"):
    keys = ["rows", "cols", "cells", "obs", "heuristic", "trials", "found_fraction", "avg_time_s", "avg_expanded", "avg_cost"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in results:
            w.writerow({k: r.get(k) for k in keys})
    print(f"CSV written to: {path}")


def main(argv=None):
    p = argparse.ArgumentParser(description="Run A* benchmarks using astar_dynamic.py module")
    p.add_argument("--sizes", nargs="*", type=int, help="Pairs of rows cols for sizes (e.g. --sizes 30 30 100 100)", default=None)
    p.add_argument("--default-sizes", action="store_true", help="Use default sizes 30x30,100x100,200x200")
    p.add_argument("--obstacle", type=float, default=0.20, help="Obstacle probability (default 0.20)")
    p.add_argument("--trials", type=int, default=3, help="Trials per configuration")
    p.add_argument("--seed", type=int, default=42, help="Master RNG seed")
    p.add_argument("--csv", type=str, default="benchmarks_runner_output.csv", help="CSV output path")
    args = p.parse_args(argv)

    if args.default_sizes or args.sizes is None or len(args.sizes) == 0:
        sizes = [(30, 30), (100, 100), (200, 200)]
    else:
        if len(args.sizes) % 2 != 0:
            p.error("--sizes requires pairs of rows and cols")
        sizes = [(args.sizes[i], args.sizes[i+1]) for i in range(0, len(args.sizes), 2)]

    results = []
    for (r, c) in sizes:
        for name, (func, w) in HEURISTICS.items():
            print(f"Running {r}x{c} heuristic {name} (trials={args.trials})...")
            res = run_single(r, c, args.obstacle, name, func, w, args.trials, args.seed)
            results.append(res)

    print_table(results)
    write_csv(results, path=args.csv)


if __name__ == "__main__":
    main()
