"""Headless harvest/survival-rate eval, separate from train.py's score-only
evaluate(). Score alone can't be read as a survival percentage (it also
includes per-action water/weed rewards) -- this tallies actual bed outcomes
(harvested/dead/spoiled) at season end instead.

Usage: python3 eval_harvest_rate.py [episodes] [policy_path_or_none]
"""
import sys

from config import NUM_AGENTS, FPS, POLICY_WEIGHTS_PATH
from grid import Grid
from agent import Agent
from rl_policy import TaskPolicy

DT = 1.0 / FPS

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 50
POLICY_ARG = sys.argv[2] if len(sys.argv) > 2 else POLICY_WEIGHTS_PATH   # pass "none" for random baseline


def run_episode(policy):
    grid   = Grid()
    agents = [Agent(i, cell, policy=policy) for i, cell in enumerate(grid.spawn_points(NUM_AGENTS))]
    grid.agents = agents
    while not grid.season_over:
        grid.tick(DT)
        for agent in agents:
            agent.update(DT, grid)
    return grid.stats()["status_counts"]


def main():
    policy = None
    label  = "random baseline"
    if POLICY_ARG.lower() != "none":
        policy = TaskPolicy.load(POLICY_ARG)
        policy.epsilon = 0.0   # greedy
        label = f"trained policy ({POLICY_ARG})"

    totals = {"growing": 0, "ready": 0, "harvested": 0, "dead": 0, "spoiled": 0}
    for ep in range(1, N_EPISODES + 1):
        counts = run_episode(policy)
        for k, v in counts.items():
            totals[k] += v

    total_beds = sum(totals.values())
    harvested, dead, spoiled = totals["harvested"], totals["dead"], totals["spoiled"]

    print(f"{label} -- {N_EPISODES} episodes, {total_beds} total beds")
    print(f"  harvested: {harvested:5d}  ({100*harvested/total_beds:.1f}%)")
    print(f"  dead:      {dead:5d}  ({100*dead/total_beds:.1f}%)")
    print(f"  spoiled:   {spoiled:5d}  ({100*spoiled/total_beds:.1f}%)")
    print(f"  survival (harvested+spoiled, i.e. not dead): {100*(total_beds-dead)/total_beds:.1f}%")


if __name__ == "__main__":
    main()
