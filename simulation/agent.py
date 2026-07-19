import math
import random
import pygame
from config import (AGENT_SIZE, AGENT_SPEED, BUSY_CHANCE, BUSY_DURATION,
                    COLORS, TASK_DURATIONS, WATER_REFILL_AMOUNT, WATER_CAPACITY, STUCK_REROUTE_FRAMES,
                    HARVEST_REWARD, WATER_REWARD, WEED_REWARD, DEATH_PENALTY)



class Agent:
    def __init__(self, agent_id, spawn_cell, policy=None):
        self.agent_id = agent_id
        self.policy   = policy   # None -> flat random.choice baseline; else a TaskPolicy
        x, y = spawn_cell.center()
        self.x, self.y = float(x), float(y)

        # Grid-stepped movement — occupancy is hard-exclusive, every cell type
        self.current_cell    = spawn_cell
        self._moving_to_cell = None   # reserved destination cell, mid-step
        self._target_cell    = None   # ultimate goal: a claimed task cell, or a wander destination
        self._path           = None   # cached BFS route (remaining cells) toward _path_target
        self._path_target    = None   # which target _path was computed for
        self._stuck_frames   = 0      # consecutive frames blocked on the same cached step
        spawn_cell.occupy(agent_id)

        # Task state
        self._task       = None   # None | "moving_to_task" | "performing_task"
        self._task_type  = None   # "monitor" | "weed" | "water" | "obtain_water"
        self._task_timer = 0.0

        # Water resource — one water-cell visit is worth WATER_CAPACITY
        # waterings before another trip is needed (>1 specifically to cut
        # down how often watering requires a full round-trip relative to
        # weeding, which has no such travel cost). Starts full.
        self.water_charges = WATER_CAPACITY

        # Harvest carrying capacity — one at a time, mirrors the water flag:
        # while True, delivering to the harvest box is the only action considered.
        self.carrying_harvest = False

        # Busy state
        self.busy       = False
        self.busy_timer = 0.0

        # RL bookkeeping — set by TaskPolicy.select()/Agent._complete_task,
        # read by train.py. Meaningless (and unused) on the random-choice path.
        self.last_state  = None
        self.last_action = None
        self.last_reward = None

        # Team-level death hits accumulate here (grid.py, on cell death) and
        # ride along on whatever real task-completion reward resolves next --
        # keeps last_reward's "task actually finished" timing intact instead
        # of forcing a Q-transition closed mid-travel.
        self.pending_penalty = 0.0

        # Longer-lived twin of last_state/last_action — set by TaskPolicy.select()
        # at the same time, but NOT cleared by train.py's per-tick bookkeeping.
        # Read by _should_yield() to arbitrate a swap-deadlock conflict against
        # the Q-value of what this agent is *currently* committed to doing;
        # last_action alone goes stale mid-episode during training once it's
        # been consumed into a Q-learning transition.
        self.committed_state  = None
        self.committed_action = None

    # ------------------------------------------------------------------
    # Movement — grid-stepped, occupancy-reserved
    # ------------------------------------------------------------------

    def _move_toward(self, tx, ty, speed):
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        # Snap once the remaining distance is within one step -- a fixed
        # threshold smaller than `speed` lets a single step overshoot the
        # target and bounce back past it forever (never landing inside the
        # threshold band), which is exactly what TIME_SCALE-driven AGENT_SPEED
        # increases exposed.
        if dist <= speed:
            self.x, self.y = tx, ty
            return True
        self.x += (dx / dist) * speed
        self.y += (dy / dist) * speed
        return False

    def step_toward(self, grid, target_cell):
        """Advance one grid-step toward target_cell, following a cached BFS
        route (recomputed only when the target changes or the route runs out —
        not every frame). Reserves the next cell the moment it commits to it
        (not on arrival), so two agents can't both start converging on the same
        empty cell mid-step. A temporarily-occupied next cell just waits and
        retries — the cached route only needs recomputing around permanent
        (blocked) obstacles, not other agents passing through. Returns True
        once arrived."""
        if self.current_cell is target_cell:
            self._path = None
            self._stuck_frames = 0
            return True

        if self._moving_to_cell is None:
            if self._path is None or self._path_target is not target_cell:
                self._path         = grid.shortest_path(self.current_cell, target_cell)[1:]
                self._path_target  = target_cell
                self._stuck_frames = 0

            if not self._path:
                return False   # no route exists — wait (shouldn't happen on a connected layout)

            next_cell = self._path[0]
            if next_cell.is_occupiable():
                next_cell.occupy(self.agent_id)
                self._moving_to_cell = next_cell
                self._path.pop(0)
                self._stuck_frames = 0
            else:
                blocker = self._find_agent(grid, next_cell.occupied_by)
                mutual_swap = (
                    next_cell is target_cell and blocker is not None and
                    blocker._target_cell is self.current_cell
                )
                if mutual_swap and self._should_yield(blocker):
                    # Reroute is provably useless for a true swap (the target
                    # itself is what's occupied) — step aside immediately
                    # instead of waiting out STUCK_REROUTE_FRAMES first.
                    self._step_aside(grid)

                if self._moving_to_cell is None:
                    self._stuck_frames += 1
                    if self._stuck_frames > STUCK_REROUTE_FRAMES:
                        # Stuck on this exact cell too long (likely another agent
                        # waiting right back at us) — reroute around current traffic.
                        rerouted = grid.shortest_path(self.current_cell, target_cell, avoid_occupied=True)[1:]
                        if rerouted:
                            self._path = rerouted
                        self._stuck_frames = 0
                    return False   # temporarily blocked by another agent — wait this frame, retry next

        tx, ty = self._moving_to_cell.center()
        if self._move_toward(tx, ty, AGENT_SPEED):
            self.current_cell.vacate()
            self.current_cell = self._moving_to_cell
            self._moving_to_cell = None

        return self.current_cell is target_cell

    def _find_agent(self, grid, agent_id):
        if agent_id is None:
            return None
        for a in getattr(grid, "agents", []):
            if a.agent_id == agent_id:
                return a
        return None

    def _current_action_value(self):
        """Q-value of what this agent is currently committed to doing, or
        None if there isn't one (random-choice baseline, or mid personal
        water/harvest trip — neither is a policy decision)."""
        if self.policy is None or self.committed_state is None or self.committed_action is None:
            return None
        return float(self.policy._predict(self.committed_state)[self.committed_action])

    def _should_yield(self, other):
        """Deterministic three-tier comparison for a detected mutual swap —
        both agents evaluate this independently off shared state and always
        reach opposite conclusions, so no negotiation channel is needed for
        them to agree on who moves. Tier 1: whoever's current committed
        action has the lower learned Q-value yields (less to lose by
        backing off) — reuses the already-trained policy, no new model.
        Tier 2 (no Q-values available, e.g. the random baseline, or an exact
        tie): whoever claimed their target more recently yields — first-come-
        first-served, not identity-based. Tier 3 (everything else tied):
        agent_id, purely to guarantee exactly one of the two yields."""
        my_v, other_v = self._current_action_value(), other._current_action_value()
        if my_v is not None and other_v is not None and my_v != other_v:
            return my_v < other_v

        my_tick    = getattr(self._target_cell, "claim_tick", None)
        other_tick = getattr(other._target_cell, "claim_tick", None)
        my_tick    = my_tick if my_tick is not None else float("inf")
        other_tick = other_tick if other_tick is not None else float("inf")
        if my_tick != other_tick:
            return my_tick > other_tick

        return self.agent_id > other.agent_id

    def _step_aside(self, grid):
        """Break a mutual swap: move toward any free neighboring cell other
        than the contested target, freeing my current cell so the other
        agent (who holds priority) can advance into it. Cached path/target
        are left alone except for invalidating _path — once I've stepped
        clear, normal step_toward re-plans toward my real target on its own."""
        options = [
            n for n in grid.neighbors(self.current_cell)
            if n.is_occupiable() and n is not self._target_cell
        ]
        if not options:
            return   # boxed in — fall through to the normal stuck/reroute safety net
        detour = random.choice(options)
        detour.occupy(self.agent_id)
        self._moving_to_cell = detour
        self._path = None

    def _pick_random_target(self, grid):
        # Deadspace is a hard barrier now — never wander toward a cell that can't be entered
        self._target_cell = random.choice([c for c in grid.all_cells() if not c.blocked])

    # ------------------------------------------------------------------
    # Task evaluation — either a TaskPolicy (rl_policy.py, trained via
    # train.py) or, when self.policy is None, the original flat/unweighted
    # baseline: pick uniformly at random among currently-known needy cells.
    # ------------------------------------------------------------------

    def evaluate_and_claim(self, grid):
        tick = grid.tick_count

        if self.policy is not None:
            result = self.policy.select(self, grid)
            if result is None:
                return
            cell, task_type = result
        else:
            candidates = [
                cell for cell in grid.plant_cells()
                if cell.is_claimable(tick) and (
                    cell.needs_monitoring(tick) or cell.needs_water() or
                    cell.needs_weeding() or cell.needs_harvest()
                )
            ]
            if not candidates:
                return

            cell = random.choice(candidates)
            applicable = []
            if cell.needs_monitoring(tick):
                applicable.append("monitor")
            if cell.needs_water():
                applicable.append("water")
            if cell.needs_weeding():
                applicable.append("weed")
            if cell.needs_harvest():
                applicable.append("harvest")
            task_type = random.choice(applicable)

        # Race guard — relies on strictly-sequential per-frame agent processing
        # in main.py; breaks if the update loop is ever parallelized.
        if not cell.is_claimable(tick):
            return

        cell.claim(self.agent_id, task_type, tick)
        self._target_cell = cell
        self._task_type   = task_type
        self._task        = "moving_to_task"
        self._task_timer  = 0.0

    def _begin_refill(self, grid):
        """Out of water — head to a water cell. Picked at random, not nearest,
        so all water cells actually get used instead of everyone piling onto
        whichever one happens to be closest to the bridge exit. Not a
        plant-cell claim (nothing to release when done), just a personal
        resource trip."""
        self._target_cell = random.choice(grid.water_cells())
        self._task_type   = "obtain_water"
        self._task        = "moving_to_task"
        self._task_timer  = 0.0
        # Not a policy decision — no Q-value exists for it, don't let a
        # swap-conflict check compare against a stale prior task's value.
        self.committed_state = self.committed_action = None

    def _begin_deliver(self, grid):
        """Carrying a harvest — head straight to the harvest box to drop it
        off. Not a plant-cell claim, just a personal delivery trip, same
        shape as _begin_refill."""
        self._target_cell = grid.harvest_box_cell()
        self._task_type   = "deliver_harvest"
        self._task        = "moving_to_task"
        self._task_timer  = 0.0
        self.committed_state = self.committed_action = None

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(self, dt, grid):
        if self.busy:
            self.busy_timer -= dt
            if self.busy_timer <= 0:
                self.busy = False
                self._pick_random_target(grid)
            return

        if self._task is None:
            if self.carrying_harvest:
                # Both hands full — nothing else to consider until it's delivered.
                self._begin_deliver(grid)
            elif self.water_charges <= 0:
                # Out of water takes priority over any other task — must refill first.
                self._begin_refill(grid)
            elif random.random() < BUSY_CHANCE:
                self.busy       = True
                self.busy_timer = BUSY_DURATION + random.uniform(-1, 1)
                return
            else:
                self.evaluate_and_claim(grid)

        if self._task == "moving_to_task":
            self._do_moving_to_task(dt, grid)
        elif self._task == "performing_task":
            self._do_performing_task(dt, grid)
        else:
            if self._target_cell is None:
                self._pick_random_target(grid)
            elif self.step_toward(grid, self._target_cell):
                self._target_cell = None

    # ------------------------------------------------------------------
    # Task handlers
    # ------------------------------------------------------------------

    def _do_moving_to_task(self, dt, grid):
        cell = self._target_cell
        # obtain_water/deliver_harvest aren't plant-cell claims — nothing to validate, just travel there
        if self._task_type not in ("obtain_water", "deliver_harvest") and cell.claimed_by != self.agent_id:
            # Claim was stolen after timing out while we were still travelling
            self._release_and_reset()
            return
        if self.step_toward(grid, cell):
            self._task       = "performing_task"
            self._task_timer = 0.0

    def _do_performing_task(self, dt, grid):
        self._task_timer += dt
        if self._task_timer >= TASK_DURATIONS[self._task_type]:
            self._complete_task(grid)

    def _complete_task(self, grid):
        cell = self._target_cell
        tick = grid.tick_count
        self.last_reward = 0.0   # overwritten below for water/weed/deliver_harvest — RL bookkeeping, harmless otherwise

        if self._task_type == "obtain_water":
            self.water_charges = WATER_CAPACITY   # refilled — nothing was claimed, nothing to release
        elif self._task_type == "deliver_harvest":
            self.carrying_harvest = False   # dropped off — nothing was claimed, nothing to release
            grid.score += HARVEST_REWARD
            self.last_reward = HARVEST_REWARD
        else:
            if self._task_type == "water":
                cell.moisture = min(100.0, cell.moisture + WATER_REFILL_AMOUNT)
                self.water_charges -= 1   # consumed one charge — refill once it hits 0
                grid.score += WATER_REWARD
                self.last_reward = WATER_REWARD
            elif self._task_type == "weed":
                cell.weed_density = 0.0
                grid.score += WEED_REWARD
                self.last_reward = WEED_REWARD
            elif self._task_type == "harvest":
                cell.status = "harvested"
                self.carrying_harvest = True
            # monitor: no true-state change

            if self._task_type != "harvest":
                cell.observe(tick)   # water/weed also count as an observation — agent is right there
            cell.release()

        if self.pending_penalty != 0.0:
            self.last_reward     += self.pending_penalty
            self.pending_penalty  = 0.0

        grid.reward_log.record(tick, grid.day_count, self.agent_id, self._task_type, cell, self.last_reward)

        self._task        = None
        self._task_type   = None
        self._target_cell = None
        self._task_timer  = 0.0
        self.committed_state = self.committed_action = None

    def _release_and_reset(self):
        if self._target_cell is not None and self._target_cell.claimed_by == self.agent_id:
            self._target_cell.release()
        if self._moving_to_cell is not None:
            self._moving_to_cell.vacate()
            self._moving_to_cell = None
        self._task        = None
        self._task_type   = None
        self._target_cell = None
        self._task_timer  = 0.0
        self.committed_state = self.committed_action = None
        self._path          = None
        self._path_target   = None
        self._stuck_frames  = 0

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, surface, font):
        cx, cy = int(self.x), int(self.y)
        half   = AGENT_SIZE // 2

        if self.busy:
            color = COLORS["agent_busy"]
        elif self._task == "performing_task":
            color = COLORS["agent_push"]
        elif self._task == "moving_to_task":
            color = COLORS["agent_waiting"]
        else:
            color = COLORS["agent"]

        pygame.draw.rect(surface, color,
                         (cx - half, cy - half, AGENT_SIZE, AGENT_SIZE),
                         border_radius=4)
        pygame.draw.rect(surface, (180, 180, 180),
                         (cx - half, cy - half, AGENT_SIZE, AGENT_SIZE),
                         1, border_radius=4)

        lbl = font.render(f"A{self.agent_id}", True, (30, 30, 30))
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy - lbl.get_height() // 2))

        if self.busy:
            s = font.render("BUSY", True, (200, 80, 80))
            surface.blit(s, (cx - s.get_width() // 2, cy + half + 2))
        elif self._task_type:
            s = font.render(self._task_type, True, COLORS["text_dim"])
            surface.blit(s, (cx - s.get_width() // 2, cy + half + 2))

    def draw_task_link(self, surface):
        if self._target_cell is None or self._task is None:
            return
        cx, cy = self._target_cell.center()
        pygame.draw.line(surface, COLORS["task_link"],
                         (int(self.x), int(self.y)),
                         (int(cx), int(cy)), 2)
