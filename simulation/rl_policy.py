import pickle
import random
from collections import deque

import numpy as np
from sklearn.ensemble import RandomForestRegressor

from config import (GRID_COLS, GRID_ROWS, STALENESS_THRESHOLD_TICKS, WATER_CAPACITY,
                    MOISTURE_DECAY_BASE, FPS)

TASK_TYPES  = ["monitor", "water", "weed", "harvest"]
N_TASKS     = len(TASK_TYPES)
N_CELLS     = 12   # must match PLANT_REGION's cell count (config.py)
FEATURE_DIM = N_CELLS * 7 + 2 + 4   # per-cell + self flags + swarm-context block
ACTION_DIM  = N_CELLS * N_TASKS
_DIST_NORM  = float(GRID_COLS + GRID_ROWS)
# Average per-tick moisture decay (jitter is 0.5x-1.5x, averages to 1.0x) --
# lets a cell's staleness be turned into a legitimate inference ("I saw this
# at 35 a while ago, it's probably near zero by now") rather than the model
# having to treat every stale cell as equally uncertain.
_MOISTURE_DECAY_PER_TICK = MOISTURE_DECAY_BASE / FPS

REPLAY_BUFFER_CAP = 8000   # bounds refit() cost as episodes accumulate
N_ESTIMATORS       = 40


def _cell_features(cell, agent, tick):
    known_moisture = cell.known_moisture / 100.0 if cell.known_moisture is not None else -1.0
    known_weed     = cell.known_weed_density / 100.0 if cell.known_weed_density is not None else -1.0
    active         = 1.0 if cell.status in ("growing", "ready") else 0.0
    staleness      = min(1.0, max(0.0, (tick - cell.last_monitored) / STALENESS_THRESHOLD_TICKS))
    dist = (abs(cell.col - agent.current_cell.col) + abs(cell.row - agent.current_cell.row)) / _DIST_NORM
    claimed_other = 1.0 if (cell.claimed_by is not None and cell.claimed_by != agent.agent_id) else 0.0
    if cell.known_moisture is None:
        est_moisture = -1.0
    else:
        ticks_since = max(0.0, tick - cell.last_monitored)
        est_moisture = max(0.0, cell.known_moisture - _MOISTURE_DECAY_PER_TICK * ticks_since) / 100.0
    return [known_moisture, known_weed, active, staleness, dist, claimed_other, est_moisture]


def _swarm_features(agent, grid):
    """Fixed-size (4-dim) summary of every OTHER agent, regardless of how
    many exist — this shape never changes with swarm size, so the same
    architecture works whether grid.agents has 2 entries or 20. Falls back
    to "no teammates nearby" defaults if agent is alone (NUM_AGENTS=1)."""
    teammates = [a for a in getattr(grid, "agents", []) if a is not agent]
    if not teammates:
        return [1.0, 0.0, 0.0, 0.0]

    dists = [
        (abs(t.current_cell.col - agent.current_cell.col) +
         abs(t.current_cell.row - agent.current_cell.row)) / _DIST_NORM
        for t in teammates
    ]
    min_dist          = min(dists)
    frac_idle         = sum(1 for t in teammates if t._task is None) / len(teammates)
    frac_out_of_water = sum(1 for t in teammates if t.water_charges <= 0) / len(teammates)
    frac_carrying     = sum(1 for t in teammates if t.carrying_harvest) / len(teammates)
    return [min_dist, frac_idle, frac_out_of_water, frac_carrying]


def build_observation(agent, grid):
    """Feature vector: 7 features per plant cell (stable grid.plant_cells()
    order) + 2 agent-own flags (water charge level, carrying_harvest) + 4
    swarm-context aggregates (nearest-teammate distance, fraction
    idle/out-of-water/carrying-harvest) — fixed shape regardless of how
    many agents exist, so a policy trained at NUM_AGENTS=2 and one trained
    at NUM_AGENTS=5 are the same architecture, just different weights."""
    tick = grid.tick_count
    feats = []
    for cell in grid.plant_cells():
        feats.extend(_cell_features(cell, agent, tick))
    feats.append(agent.water_charges / WATER_CAPACITY)   # 0..1, not just binary now that capacity > 1
    feats.append(1.0 if agent.carrying_harvest else 0.0)
    feats.extend(_swarm_features(agent, grid))
    return np.array(feats, dtype=np.float32)


def build_action_mask(agent, grid):
    """ACTION_DIM-length bool mask (N_CELLS * N_TASKS), mirroring the exact
    predicates evaluate_and_claim uses — the policy can never pick an action
    the existing rules wouldn't already consider valid."""
    tick = grid.tick_count
    mask = np.zeros(ACTION_DIM, dtype=bool)
    for i, cell in enumerate(grid.plant_cells()):
        if not cell.is_claimable(tick):
            continue
        needs = {
            "monitor": cell.needs_monitoring(tick),
            "water":   cell.needs_water(),
            "weed":    cell.needs_weeding(),
            "harvest": cell.needs_harvest(),
        }
        for j, task in enumerate(TASK_TYPES):
            if needs[task]:
                mask[i * N_TASKS + j] = True
    return mask


def decode_action(action_idx, grid):
    cell_idx, task_idx = divmod(action_idx, N_TASKS)
    return grid.plant_cells()[cell_idx], TASK_TYPES[task_idx]


class TaskPolicy:
    def __init__(self, epsilon=0.1, gamma=0.95):
        self.epsilon    = epsilon
        self.gamma      = gamma
        self.is_fitted  = False
        self.model = RandomForestRegressor(
            n_estimators=N_ESTIMATORS,
            n_jobs=-1,
        )
        # Trees don't support incremental partial_fit — transitions accumulate
        # here and get bootstrapped/refit in one batch per episode (refit()),
        # classic Fitted Q-Iteration instead of online SGD.
        self.buffer = deque(maxlen=REPLAY_BUFFER_CAP)

    def _predict(self, obs):
        if not self.is_fitted:
            return np.zeros(ACTION_DIM, dtype=np.float32)
        return self.model.predict(obs.reshape(1, -1))[0]

    def select(self, agent, grid):
        """Chooses (cell, task_type) for an idle agent, or None if nothing is
        valid right now. Side effect: stashes agent.last_state/last_action so
        train.py can build a training pair once this decision resolves."""
        obs  = build_observation(agent, grid)
        mask = build_action_mask(agent, grid)
        if not mask.any():
            return None

        valid_indices = np.flatnonzero(mask)
        if not self.is_fitted or random.random() < self.epsilon:
            action = int(random.choice(valid_indices))
        else:
            q = self._predict(obs)
            q_masked = np.where(mask, q, -np.inf)
            action = int(np.argmax(q_masked))

        agent.last_state  = obs
        agent.last_action = action
        # Longer-lived twin, read by Agent._should_yield() to arbitrate a
        # swap-deadlock conflict — unlike last_state/last_action, train.py
        # never clears these, so they stay valid for as long as this
        # decision is actually being acted on (not just until the next
        # Q-learning transition is captured).
        agent.committed_state  = obs
        agent.committed_action = action
        return decode_action(action, grid)

    def update(self, s, a, r, s2, mask2, done):
        """Buffers one resolved decision for the next refit() — no per-step
        fitting (trees can't do that cheaply); train.py calls refit() once
        per episode instead."""
        self.buffer.append((s, a, r, s2, mask2, done))

    def refit(self):
        """One batch Fitted-Q-Iteration step over the whole replay buffer:
        bootstrap targets off the model as it stood before this call (or
        zeros, if never fitted yet), then refit the forest in a single
        batched call. Call once per training episode, not per decision —
        this is batch-mode RL, not online SGD."""
        if not self.buffer:
            return

        S  = np.stack([t[0] for t in self.buffer])
        A  = np.array([t[1] for t in self.buffer])
        R  = np.array([t[2] for t in self.buffer], dtype=np.float32)
        S2 = np.stack([t[3] for t in self.buffer])
        M2 = np.stack([t[4] for t in self.buffer])
        D  = np.array([t[5] for t in self.buffer], dtype=bool)

        n = len(self.buffer)
        if self.is_fitted:
            Q  = self.model.predict(S)
            Q2 = self.model.predict(S2)
        else:
            Q  = np.zeros((n, ACTION_DIM), dtype=np.float32)
            Q2 = np.zeros((n, ACTION_DIM), dtype=np.float32)

        Q2_masked   = np.where(M2, Q2, -np.inf)
        has_valid   = M2.any(axis=1)
        max_q2      = np.where(has_valid, np.max(Q2_masked, axis=1), 0.0)
        targets     = R + self.gamma * max_q2 * (~D)

        Y = Q.copy()
        Y[np.arange(n), A] = targets

        self.model.fit(S, Y)
        self.is_fitted = True

    def save(self, path):
        with open(path, "wb") as f:
            pickle.dump({"model": self.model, "is_fitted": self.is_fitted}, f)

    @classmethod
    def load(cls, path):
        with open(path, "rb") as f:
            data = pickle.load(f)
        policy = cls(epsilon=0.0)
        policy.model      = data["model"]
        policy.is_fitted  = data["is_fitted"]
        return policy
