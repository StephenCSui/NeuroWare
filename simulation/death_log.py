"""Per-death log — records which bed died, when, and why (dehydration,
weeds, or both), for diagnosing whether difficulty tuning is killing beds
for the reason intended rather than just an aggregate death count."""

import csv


class DeathLog:
    def __init__(self):
        self.events = []   # append-only list of dicts, insertion order == chronological

    def record(self, tick, day, cell, cause):
        self.events.append({
            "tick":  tick,
            "day":   day,
            "cell":  f"{cell.col},{cell.row}",
            "cause": cause,   # "dehydration" | "weeds" | "both"
        })

    def summary(self):
        counts = {"dehydration": 0, "weeds": 0, "both": 0}
        for e in self.events:
            counts[e["cause"]] += 1
        return {"count": len(self.events), **counts}

    def save(self, path):
        if not self.events:
            return
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.events[0].keys()))
            writer.writeheader()
            writer.writerows(self.events)
