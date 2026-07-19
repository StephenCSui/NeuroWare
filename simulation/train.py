"""Headless training loop for the task-prioritization policy. No pygame, no
real-time pacing -- runs full 30-day seasons as fast as Python allows.

Usage: python3 train.py [episodes]
"""
import os
import sys

from config import NUM_AGENTS, FPS, POLICY_WEIGHTS_PATH
from grid import Grid
from agent import Agent
from rl_policy import TaskPolicy, build_observation, build_action_mask

DT           = 1.0 / FPS   # must match runtime dt -- decay/sprout/task durations are all dt-scaled
NUM_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
# Optional agent-count override -- proves the observation/action shapes in
# rl_policy.py don't hard-code a specific swarm size (see build_observation's
# swarm-context block). Training itself is still per-agent-count: a policy
# trained here is only valid for this many agents.
TRAIN_NUM_AGENTS = int(sys.argv[2]) if len(sys.argv) > 2 else NUM_AGENTS
EVAL_EPISODES = 50
LOG_EVERY    = int(sys.argv[3]) if len(sys.argv) > 3 else min(100, max(1, NUM_EPISODES // 8))


def run_episode(policy, train=True, num_agents=None):
    """Plays one season. If train, buffers Q-learning transitions as decisions
    resolve and refits the policy once at the end of the episode, returning
    the final score. If not train, policy acts greedily (or randomly, if
    policy is None) with no learning -- used for eval."""
    if num_agents is None:
        num_agents = TRAIN_NUM_AGENTS
    grid   = Grid()
    agents = [Agent(i, cell, policy=policy) for i, cell in enumerate(grid.spawn_points(num_agents))]
    grid.agents = agents   # lets build_observation see the rest of the swarm
    # Eval mode with a real policy: force greedy (epsilon=0) for the episode,
    # restore afterward so training's own schedule isn't disturbed.
    saved_epsilon = None
    if policy is not None and not train:
        saved_epsilon = policy.epsilon
        policy.epsilon = 0.0

    pending = {a.agent_id: None for a in agents}   # agent_id -> (state, action) awaiting resolution

    while not grid.season_over:
        grid.tick(DT)
        for agent in agents:
            agent.update(DT, grid)

            if train and agent.last_reward is not None:
                if pending[agent.agent_id] is not None:
                    s, a = pending[agent.agent_id]
                    s2   = build_observation(agent, grid)
                    mask2 = build_action_mask(agent, grid)
                    policy.update(s, a, agent.last_reward, s2, mask2, grid.season_over)
                pending[agent.agent_id] = None
                agent.last_reward = None

            if train and agent.last_action is not None:
                pending[agent.agent_id] = (agent.last_state, agent.last_action)
                agent.last_action = None

    if saved_epsilon is not None:
        policy.epsilon = saved_epsilon

    if train and policy is not None:
        policy.refit()

    return grid.score


def evaluate(policy, n=EVAL_EPISODES):
    scores = [run_episode(policy, train=False) for _ in range(n)]
    return sum(scores) / len(scores)


def main():
    policy = TaskPolicy(epsilon=1.0)
    print(f"training with {TRAIN_NUM_AGENTS} agent(s)")

    for ep in range(1, NUM_EPISODES + 1):
        policy.epsilon = max(0.05, 1.0 - ep / NUM_EPISODES)
        score = run_episode(policy, train=True)
        if ep % LOG_EVERY == 0:
            print(f"episode {ep:5d}/{NUM_EPISODES}  score={score:6.1f}  "
                  f"epsilon={policy.epsilon:.3f}  buffer={len(policy.buffer)}")

    os.makedirs(os.path.dirname(POLICY_WEIGHTS_PATH), exist_ok=True)
    policy.save(POLICY_WEIGHTS_PATH)
    print(f"saved policy to {POLICY_WEIGHTS_PATH}")

    print("\nevaluating (greedy trained policy vs. random baseline)...")
    trained_avg  = evaluate(policy)
    baseline_avg = evaluate(None)
    print(f"trained policy avg score over {EVAL_EPISODES} seasons: {trained_avg:.1f}")
    print(f"random baseline avg score over {EVAL_EPISODES} seasons: {baseline_avg:.1f}")


if __name__ == "__main__":
    main()
