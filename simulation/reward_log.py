"""Per-event reward log — records every reward an agent actually receives,
the instant it's known (see Agent._complete_task), rather than just the
aggregate grid.score. Not consumed by anything yet; groundwork for
debugging reward shaping / verifying attribution later."""

import csv


class RewardLog:
    def __init__(self):
        self.events = []   # append-only list of dicts, insertion order == chronological

    def record(self, tick, day, agent_id, task_type, cell, reward):
        self.events.append({
            "tick":      tick,
            "day":       day,
            "agent_id":  agent_id,
            "task_type": task_type,
            "cell":      f"{cell.col},{cell.row}" if cell is not None else "",
            "reward":    reward,
            "sign":      "positive" if reward > 0 else ("negative" if reward < 0 else "neutral"),
        })

    def summary(self):
        total = sum(e["reward"] for e in self.events)
        return {
            "count":    len(self.events),
            "positive": sum(1 for e in self.events if e["reward"] > 0),
            "negative": sum(1 for e in self.events if e["reward"] < 0),
            "neutral":  sum(1 for e in self.events if e["reward"] == 0),
            "total":    total,
        }

    def save(self, path):
        if not self.events:
            return
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.events[0].keys()))
            writer.writeheader()
            writer.writerows(self.events)
