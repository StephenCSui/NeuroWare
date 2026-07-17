import pickle
import random

import numpy as np
from sklearn.neural_network import MLPRegressor

from config import GRID_COLS, GRID_ROWS, STALENESS_THRESHOLD_TICKS

TASK_TYPES  = ["monitor", "water", "weed", "harvest"]
N_TASKS     = len(TASK_TYPES)
N_CELLS     = 6
FEATURE_DIM = N_CELLS * 6 + 2
ACTION_DIM  = N_CELLS * N_TASKS
_DIST_NORM  = float(GRID_COLS + GRID_ROWS)


def _cell_features(cell, agent, tick):
    known_moisture = cell.known_moisture / 100.0 if cell.known_moisture is not None else -1.0
    known_weed     = cell.known_weed_density / 100.0 if cell.known_weed_density is not None else -1.0
    active         = 1.0 if cell.status in ("growing", "ready") else 0.0
    staleness      = min(1.0, max(0.0, (tick - cell.last_monitored) / STALENESS_THRESHOLD_TICKS))
    dist = (abs(cell.col - agent.current_cell.col) + abs(cell.row - agent.current_cell.row)) / _DIST_NORM
    claimed_other = 1.0 if (cell.claimed_by is not None and cell.claimed_by != agent.agent_id) else 0.0
    return [known_moisture, known_weed, active, staleness, dist, claimed_other]


def build_observation(agent, grid):
    """38-dim feature vector: 6 features per plant cell (stable grid.plant_cells()
    order) + 2 agent-own flags (water, carrying_harvest)."""
    tick = grid.tick_count
    feats = []
    for cell in grid.plant_cells():
        feats.extend(_cell_features(cell, agent, tick))
    feats.append(1.0 if agent.water else 0.0)
    feats.append(1.0 if agent.carrying_harvest else 0.0)
    return np.array(feats, dtype=np.float32)


def build_action_mask(agent, grid):
    """24-dim bool mask, mirroring the exact predicates evaluate_and_claim uses —
    the policy can never pick an action the existing rules wouldn't already
    consider valid."""
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
        self.model = MLPRegressor(
            hidden_layer_sizes=(64, 64),
            activation="relu",
            solver="adam",
            learning_rate_init=1e-3,
            max_iter=1,       # one partial_fit call = one gradient step
            warm_start=True,
        )

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
        return decode_action(action, grid)

    def update(self, s, a, r, s2, mask2, done):
        """One Q-learning step: target = r + gamma * max(Q(s2)[valid]), or just
        r if the episode ended or s2 has no valid actions left."""
        target = r
        if not done and mask2.any():
            q2 = self._predict(s2)
            target = r + self.gamma * float(np.max(np.where(mask2, q2, -np.inf)))

        target_vec = self._predict(s).copy()
        target_vec[a] = target
        self.model.partial_fit(s.reshape(1, -1), target_vec.reshape(1, -1))
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
