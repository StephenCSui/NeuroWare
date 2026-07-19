import random
import heapq
import pygame
from reward_log import RewardLog
from death_log import DeathLog
from config import (
    CELL_SIZE, GRID_COLS, GRID_ROWS,
    HARVEST_BOX, WATER_REGION, PLANT_REGION,
    MOISTURE_LOW_THRESHOLD, WEED_HIGH_THRESHOLD, STALENESS_THRESHOLD_TICKS,
    MOISTURE_DECAY_BASE, WEED_SPROUT_CHANCE, WEED_SPROUT_MIN, WEED_SPROUT_MAX,
    CLAIM_TIMEOUT_TICKS, COLORS,
    DAY_LENGTH_TICKS, DAYS_TO_MATURE, SEASON_LENGTH_DAYS, DEATH_PENALTY,
    DEHYDRATION_BLEED_THRESHOLD, DEHYDRATION_BLEED_RATE,
)


class Cell:
    def __init__(self, col, row, cell_type):
        self.col       = col
        self.row       = row
        self.cell_type = cell_type   # "plant" | "walkway" | "water" | "harvest_box"

        # Physical occupancy — every cell type, hard-exclusive, independent of task claiming
        self.occupied_by = None
        self.blocked      = False   # no cell type is a hard barrier at this scale — kept for future use

        if cell_type == "plant":
            # True state — hidden, drifts on its own regardless of whether anyone's looked
            self.moisture     = random.uniform(40.0, 80.0)
            self.weed_density = random.uniform(0.0, 15.0)

            # Lifecycle — "growing" -> "ready" -> "harvested" | "dead" | "spoiled"
            # (last three terminal: true-state drift stops, cell never respawns this season)
            self.status = "growing"

            # Known state — the swarm's belief, only updated by observe()
            self.known_moisture     = None
            self.known_weed_density = None
            # Staggered, not 0 — every cell starting at the same last_monitored
            # would make them all cross the staleness threshold on the same
            # tick, forcing a synchronized monitor-everything burst followed
            # by a synchronized quiet gap instead of continuous small activity.
            self.last_monitored = -random.uniform(0, STALENESS_THRESHOLD_TICKS)

            # Task-work claim — distinct from occupied_by
            self.claimed_by = None
            self.claim_task = None
            self.claim_tick = None
        else:
            self.moisture = self.weed_density = None
            self.status = None
            self.known_moisture = self.known_weed_density = None
            self.last_monitored = None
            self.claimed_by = self.claim_task = self.claim_tick = None

    def center(self):
        return (self.col * CELL_SIZE + CELL_SIZE / 2,
                self.row * CELL_SIZE + CELL_SIZE / 2)

    # ------------------------------------------------------------------
    # Occupancy — physical presence, every cell type
    # ------------------------------------------------------------------

    def is_occupiable(self):
        return not self.blocked and self.occupied_by is None

    def occupy(self, agent_id):
        self.occupied_by = agent_id

    def vacate(self):
        self.occupied_by = None

    # ------------------------------------------------------------------
    # Needs — plant cells only, evaluated against KNOWN state
    # ------------------------------------------------------------------

    def _is_active(self):
        # Only a still-growing plant needs maintenance or drifts at all --
        # once "ready" it's frozen (no water/weed/monitor needs, no decay,
        # no neglect-death risk) for the whole harvest window, the only
        # thing left to do with it is collect it. Terminal cells
        # (harvested/dead/spoiled) are done for the season either way.
        return self.cell_type == "plant" and self.status == "growing"

    def needs_water(self):
        if not self._is_active() or self.known_moisture is None:
            return False
        return self.known_moisture < MOISTURE_LOW_THRESHOLD

    def needs_weeding(self):
        if not self._is_active() or self.known_weed_density is None:
            return False
        return self.known_weed_density > WEED_HIGH_THRESHOLD

    def needs_monitoring(self, tick):
        if not self._is_active():
            return False
        return (tick - self.last_monitored) > STALENESS_THRESHOLD_TICKS

    def needs_harvest(self):
        return self.cell_type == "plant" and self.status == "ready"

    # ------------------------------------------------------------------
    # Task-work claim — plant cells only
    # ------------------------------------------------------------------

    def is_claimable(self, tick):
        if self.claimed_by is None:
            return True
        return (tick - self.claim_tick) > CLAIM_TIMEOUT_TICKS

    def claim(self, agent_id, task_type, tick):
        self.claimed_by = agent_id
        self.claim_task = task_type
        self.claim_tick = tick

    def release(self):
        self.claimed_by = None
        self.claim_task = None
        self.claim_tick = None

    def observe(self, tick):
        """Copy true state into known state. The only place known state changes."""
        self.known_moisture     = self.moisture
        self.known_weed_density = self.weed_density
        self.last_monitored     = tick

    # ------------------------------------------------------------------
    # True-state evolution — stochastic, not smooth/linear
    # ------------------------------------------------------------------

    def tick(self, dt):
        if not self._is_active():
            return   # terminal cells are frozen — no drift, no respawn
        self.moisture = max(0.0, self.moisture - random.uniform(0.5, 1.5) * MOISTURE_DECAY_BASE * dt)
        if random.random() < WEED_SPROUT_CHANCE * dt:
            self.weed_density = min(100.0, self.weed_density +
                                     random.uniform(WEED_SPROUT_MIN, WEED_SPROUT_MAX))

    # ------------------------------------------------------------------
    # Draw — reflects TRUE state (simulation's own god-view), except the
    # unknown tint which reflects the swarm's ignorance until first observed
    # ------------------------------------------------------------------

    def color(self):
        if self.cell_type == "walkway":
            return COLORS["walkway"]
        if self.cell_type == "harvest_box":
            return COLORS["harvest_box"]
        if self.cell_type == "water":
            return COLORS["water"]
        if self.status in ("harvested", "dead", "spoiled"):
            return COLORS[self.status]
        if self.status == "growing" and self.moisture <= DEHYDRATION_BLEED_THRESHOLD:
            return COLORS["bleeding"]   # true state, overrides "unknown" too -- this is for the human watching, not the swarm's own knowledge
        if self.known_moisture is None:
            return COLORS["unknown"]

        def lerp(c1, c2, t):
            return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))

        need = max(0.0, min(1.0, 1.0 - self.moisture / 100.0))       # 0=full, 1=needs water
        sev  = max(0.0, min(1.0, self.weed_density / 100.0))         # 0=clean, 1=very weedy

        c = lerp(COLORS["healthy"], COLORS["needs_water"], need)
        c = lerp(c, COLORS["weedy"], sev)
        return c


class Grid:
    def __init__(self):
        self.cols        = GRID_COLS
        self.rows        = GRID_ROWS
        self.tick_count  = 0
        self.day_count   = 0
        self.season_over = False
        self.score       = 0.0
        self.reward_log  = RewardLog()
        self.death_log   = DeathLog()
        self.cells       = self._build_layout()

    def _build_layout(self):
        """Template-based layout: base fill is walkway (pathway), with water
        / plant / harvest_box regions stamped on top in that order (plant
        last among the regions, so it wins if any were ever adjusted to
        overlap; harvest_box is a single cell stamped last of all)."""
        cells = [[Cell(col, row, "walkway") for row in range(self.rows)]
                 for col in range(self.cols)]

        def stamp(region, cell_type):
            rcol, rrow, rw, rh = region
            for col in range(rcol, rcol + rw):
                for row in range(rrow, rrow + rh):
                    if 0 <= col < self.cols and 0 <= row < self.rows:
                        cells[col][row] = Cell(col, row, cell_type)

        stamp(WATER_REGION, "water")
        stamp(PLANT_REGION, "plant")
        hcol, hrow = HARVEST_BOX
        cells[hcol][hrow] = Cell(hcol, hrow, "harvest_box")

        return cells

    # ------------------------------------------------------------------
    # Lookup / topology
    # ------------------------------------------------------------------

    def cell_at(self, col, row):
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return self.cells[col][row]
        return None

    def cell_at_px(self, x, y):
        return self.cell_at(int(x // CELL_SIZE), int(y // CELL_SIZE))

    def neighbors(self, cell):
        """Orthogonally-adjacent in-bounds cells."""
        out = []
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = self.cell_at(cell.col + dc, cell.row + dr)
            if n is not None:
                out.append(n)
        return out

    def all_cells(self):
        return [c for column in self.cells for c in column]

    def shortest_path(self, start, target, avoid_occupied=False):
        """Dijkstra over non-blocked cells with uniform per-cell cost (no
        traversal weighting — the map's small and open enough at this scale
        that plain shortest-path is sufficient). By default ignores dynamic
        agent occupancy (the caller waits/retries on a temporarily-occupied
        next cell rather than replanning every frame). `avoid_occupied=True`
        additionally treats currently-occupied cells as impassable (except
        the target itself) — used as a fallback when a cached route has been
        stuck on the same contested cell too long, so two agents can't wait
        on each other forever. Returns [start,...,target], or [] if
        unreachable."""
        if start is target:
            return [start]
        dist    = {start: 0}
        parent  = {}
        visited = set()
        heap    = [(0, id(start), start)]   # id() as tiebreaker — Cells aren't orderable
        while heap:
            d, _, cur = heapq.heappop(heap)
            if cur in visited:
                continue
            visited.add(cur)
            if cur is target:
                break
            for n in self.neighbors(cur):
                if n.blocked or n in visited:
                    continue
                if avoid_occupied and n is not target and not n.is_occupiable():
                    continue
                nd = d + 1
                if n not in dist or nd < dist[n]:
                    dist[n]   = nd
                    parent[n] = cur
                    heapq.heappush(heap, (nd, id(n), n))
        if target not in visited:
            return []
        path = [target]
        while path[-1] is not start:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def plant_cells(self):
        return [c for column in self.cells for c in column if c.cell_type == "plant"]

    def walkway_cells(self):
        return [c for column in self.cells for c in column if c.cell_type == "walkway"]

    def water_cells(self):
        return [c for column in self.cells for c in column if c.cell_type == "water"]

    def harvest_box_cell(self):
        for column in self.cells:
            for c in column:
                if c.cell_type == "harvest_box":
                    return c
        return None

    def spawn_points(self, n):
        """n distinct cells for agent spawning — prefer walkways so agents
        don't start out occupying (and blocking work on) plant cells."""
        candidates = self.walkway_cells()
        if len(candidates) < n:
            candidates += self.plant_cells()
        random.shuffle(candidates)
        return candidates[:n]

    # ------------------------------------------------------------------
    # Simulation step
    # ------------------------------------------------------------------

    def tick(self, dt):
        if self.season_over:
            return   # frozen — agents already halted, and nothing left to drift
        self.tick_count += 1
        self.day_count   = self.tick_count // DAY_LENGTH_TICKS
        for cell in self.plant_cells():
            cell.tick(dt)
            if cell.status == "growing":
                if cell.moisture <= DEHYDRATION_BLEED_THRESHOLD:
                    bleed = DEHYDRATION_BLEED_RATE * dt
                    self.score += bleed
                    for agent in self.agents:
                        agent.pending_penalty += bleed

                # Death is only a growing-phase risk now -- once "ready",
                # true state is frozen (Cell._is_active()), so neither
                # condition below can even trigger during the harvest window.
                dehydrated = cell.moisture <= 0.0
                overweeded = cell.weed_density >= 100.0
                both_bad   = cell.moisture <= 20.0 and cell.weed_density >= 80.0
                if dehydrated or overweeded or both_bad:
                    if dehydrated and overweeded:
                        cause = "both"
                    elif dehydrated:
                        cause = "dehydration"
                    elif overweeded:
                        cause = "weeds"
                    else:
                        cause = "both"   # only the combined (both_bad) threshold fired
                    cell.status = "dead"
                    self.death_log.record(self.tick_count, self.day_count, cell, cause)
                    self.score += DEATH_PENALTY
                    for agent in self.agents:
                        agent.pending_penalty += DEATH_PENALTY
                elif self.day_count >= DAYS_TO_MATURE:
                    cell.status = "ready"
        if self.day_count >= SEASON_LENGTH_DAYS:
            self.season_over = True
            for cell in self.plant_cells():
                if cell.status == "ready":
                    cell.status = "spoiled"

    def stats(self):
        cells = self.plant_cells()
        n = len(cells)
        known_count = sum(1 for c in cells if c.known_moisture is not None)
        active = [c for c in cells if c.status in ("growing", "ready")]
        avg_moist = sum(c.moisture for c in active) / len(active) if active else 0.0
        avg_weed  = sum(c.weed_density for c in active) / len(active) if active else 0.0
        needy  = {"water": 0, "weed": 0, "monitor": 0, "harvest": 0}
        claims = {"water": 0, "weed": 0, "monitor": 0, "harvest": 0}
        status_counts = {"growing": 0, "ready": 0, "harvested": 0, "dead": 0, "spoiled": 0}
        for c in cells:
            status_counts[c.status] += 1
            if c.needs_water():
                needy["water"] += 1
            if c.needs_weeding():
                needy["weed"] += 1
            if c.needs_monitoring(self.tick_count):
                needy["monitor"] += 1
            if c.needs_harvest():
                needy["harvest"] += 1
            if c.claim_task:
                claims[c.claim_task] += 1
        return {
            "avg_moisture":  avg_moist,
            "avg_weed":      avg_weed,
            "unknown_cells": n - known_count,
            "needy":         needy,
            "claims":        claims,
            "tick":          self.tick_count,
            "day":           self.day_count,
            "season_over":   self.season_over,
            "score":         self.score,
            "status_counts": status_counts,
        }

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, surface):
        for cell in self.all_cells():
            rect = (cell.col * CELL_SIZE, cell.row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(surface, cell.color(), rect)
            if cell.cell_type == "plant" and cell.claimed_by is not None:
                pygame.draw.rect(surface, COLORS["claim_marker"], rect, 2)
