"""Headless baseline measurement for the deterministic nearest-robot
assignment (assignment.py), before any RL decision layer is introduced.
Mirrors simulation/eval_harvest_rate.py's role in the farm project: establish
real numbers first, so a later RL comparison means something.
"""
import statistics as stats
import sys

from config import FPS
from sim import Simulation


def run(sim_minutes=10):
    sim = Simulation()
    total_ticks = int(sim_minutes * 60 * FPS)
    dt = 1.0 / FPS
    for _ in range(total_ticks):
        sim.tick(dt)
    return sim.completed_records, total_ticks


def summarize(records, total_ticks):
    if not records:
        print("No objects delivered in this run -- nothing to summarize.")
        return

    fill_s     = [(f - s) / FPS for s, f, a, d in records]
    anchor_s   = [(a - f) / FPS for s, f, a, d in records]
    cycle_s    = [(d - s) / FPS for s, f, a, d in records]
    elapsed_min = total_ticks / FPS / 60.0

    def line(label, values):
        print(f"  {label:<28} mean {stats.mean(values):6.2f}s   "
              f"median {stats.median(values):6.2f}s   max {max(values):6.2f}s")

    print(f"Delivered: {len(records)} objects over {elapsed_min:.1f} sim-minutes "
          f"({len(records) / elapsed_min:.2f} objects/min)")
    line("time to fully assigned", fill_s)
    line("time assigned -> anchored", anchor_s)
    line("total cycle (spawn->delivered)", cycle_s)


if __name__ == "__main__":
    minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    records, total_ticks = run(minutes)
    summarize(records, total_ticks)
