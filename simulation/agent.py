import math
import random
import pygame
from config import (AGENT_SIZE, AGENT_SPEED, COMM_RANGE, VISION_RANGE, COLORS,
                    BUSY_CHANCE, BUSY_DURATION, PUSH_FORCE,
                    BREAKOFF_WAIT, DEST_THRESHOLD,
                    WINDOW_WIDTH, WINDOW_HEIGHT)


class Agent:
    def __init__(self, agent_id, x, y):
        self.agent_id    = agent_id
        self.x           = float(x)
        self.y           = float(y)
        self.neighbours  = []
        self.seen_items  = []

        # Push slot — {"item": Item, "side": str, "idx": int} or None
        self.push_slot   = None
        self.going_for   = None   # item_id currently committed to

        # Task state
        self._task       = None
        self._wait_timer = 0.0   # time spent in waiting_for_co_pusher

        # Busy state
        self.busy        = False
        self.busy_timer  = 0.0

        # Gossip store — item_id → Item (destination/phase live on item object)
        self.known_tasks = {}

        # Wander target
        self._target_x   = float(x)
        self._target_y   = float(y)
        self._pick_random_target()

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def _pick_random_target(self):
        margin = 40
        self._target_x = random.uniform(margin, WINDOW_WIDTH  - margin)
        self._target_y = random.uniform(margin, WINDOW_HEIGHT - margin)

    def _move_toward(self, tx, ty, speed=None, arrive_dist=2):
        spd  = speed or AGENT_SPEED
        dx   = tx - self.x
        dy   = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < arrive_dist:
            return True
        self.x += (dx / dist) * spd
        self.y += (dy / dist) * spd
        return False

    # ------------------------------------------------------------------
    # Sensing
    # ------------------------------------------------------------------

    def sense_neighbours(self, all_agents):
        self.neighbours = [
            a for a in all_agents
            if a.agent_id != self.agent_id
            and math.hypot(a.x - self.x, a.y - self.y) <= COMM_RANGE
        ]

    def sense_items(self, all_items):
        self.seen_items = [
            item for item in all_items
            if not item.delivered
            and math.hypot(item.x - self.x, item.y - self.y) <= VISION_RANGE
        ]

    # ------------------------------------------------------------------
    # Gossip
    # ------------------------------------------------------------------

    def absorb_captain_info(self, captains):
        for cap in captains:
            if math.hypot(self.x - cap.x, self.y - cap.y) > COMM_RANGE:
                continue
            for item_id, item in cap.active_pings.items():
                if item_id not in self.known_tasks:
                    self.known_tasks[item_id] = item
            for item_id, (item, _) in cap.goal_pings.items():
                # destination/phase already set on item by captain
                self.known_tasks[item_id] = item

    def gossip(self):
        for nb in self.neighbours:
            for item_id, item in self.known_tasks.items():
                if item_id not in nb.known_tasks:
                    nb.known_tasks[item_id] = item

    def cleanup_known_tasks(self):
        for item_id in list(self.known_tasks.keys()):
            item = self.known_tasks[item_id]
            if item.delivered:
                del self.known_tasks[item_id]
            elif item.stored and item.phase == "to_store":
                # Storage leg complete — waiting for goal request; drop it
                del self.known_tasks[item_id]
            elif not item.is_available():
                # All slots filled — only keep if we hold one
                if self.push_slot is None or self.push_slot.get("item") is not item:
                    del self.known_tasks[item_id]

    # ------------------------------------------------------------------
    # Task evaluation
    # ------------------------------------------------------------------

    def evaluate_known_tasks(self):
        if self._task is not None:
            return

        in_progress = []
        fresh       = []

        for item_id, item in self.known_tasks.items():
            if not item.is_available():
                continue
            needed   = item.needed_sides()
            any_open = any(len(item.claimed_slots[s]) < item.agents_per_side for s in needed)
            if not any_open:
                continue
            has_claimed = any(len(item.claimed_slots[s]) > 0 for s in needed)
            if has_claimed:
                in_progress.append(item)
            else:
                fresh.append(item)

        # Pass 1 — fill partial teams first; within an item prefer the side closest to full
        for item in in_progress:
            needed = item.needed_sides()
            # Sort sides by how many are already claimed descending (closest to full first)
            sorted_sides = sorted(needed,
                                  key=lambda s: len(item.claimed_slots[s]),
                                  reverse=True)
            for side in sorted_sides:
                if len(item.claimed_slots[side]) < item.agents_per_side:
                    if self._try_claim_slot(item, side):
                        return

        # Pass 2 — take on fresh items
        for item in fresh:
            for side in item.needed_sides():
                if len(item.claimed_slots[side]) < item.agents_per_side:
                    if self._try_claim_slot(item, side):
                        return

    def _try_claim_slot(self, item, side):
        if item.claim_slot(self, side):
            idx             = item.claimed_slots[side].index(self)
            self.push_slot  = {"item": item, "side": side, "idx": idx}
            self._task      = "seek_push_slot"
            self.going_for  = item.item_id
            return True
        return False

    def reconsider_task(self):
        """Break off from waiting if a better use exists."""
        if self._task != "waiting_for_co_pusher":
            return
        if self._wait_timer < BREAKOFF_WAIT:
            return

        current_item = self.push_slot["item"]

        # Look for another item with urgently needed slots (partial team waiting)
        for item in self.known_tasks.values():
            if item is current_item or not item.is_available():
                continue
            needed = item.needed_sides()
            partial = any(0 < len(item.claimed_slots[s]) < item.agents_per_side
                          for s in needed)
            if partial:
                self._release_and_reset()
                return

        # After double the threshold, consider any item with open slots
        if self._wait_timer > BREAKOFF_WAIT * 2:
            for item in self.known_tasks.values():
                if item is current_item or not item.is_available():
                    continue
                if any(len(item.claimed_slots[s]) < item.agents_per_side
                       for s in item.needed_sides()):
                    self._release_and_reset()
                    return

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(self, dt, zones, captains, items):
        self.absorb_captain_info(captains)
        self.gossip()
        self.cleanup_known_tasks()

        if self.busy:
            self.busy_timer -= dt
            if self.busy_timer <= 0:
                self.busy      = False
                self._task     = None
                self.going_for = None
                self._pick_random_target()
            return

        if self._task is None and random.random() < BUSY_CHANCE:
            self.busy       = True
            self.busy_timer = BUSY_DURATION + random.uniform(-1, 1)
            return

        self.reconsider_task()
        self.evaluate_known_tasks()

        if self._task == "seek_push_slot":
            self._do_seek_push_slot(dt)
        elif self._task == "waiting_for_co_pusher":
            self._do_waiting_for_co_pusher(dt)
        elif self._task == "pushing":
            self._do_pushing(dt, zones, captains)
        elif self._task is None:
            arrived = self._move_toward(self._target_x, self._target_y)
            if arrived:
                self._pick_random_target()

        self._resolve_item_collisions(items)

        self.x = max(10, min(WINDOW_WIDTH  - 10, self.x))
        self.y = max(10, min(WINDOW_HEIGHT - 10, self.y))

    # ------------------------------------------------------------------
    # Task handlers
    # ------------------------------------------------------------------

    def _do_seek_push_slot(self, dt):
        if self.push_slot is None:
            self._reset()
            return
        item = self.push_slot["item"]
        side = self.push_slot["side"]

        if item.delivered or (item.stored and item.phase == "to_store"):
            self._release_and_reset()
            return

        # Recompute idx in case list changed
        try:
            idx = item.claimed_slots[side].index(self)
        except ValueError:
            self._release_and_reset()
            return

        self.push_slot["idx"] = idx
        tx, ty = item.slot_position(side, idx)

        # Speed is determined by what agents can observe about the perpendicular side
        perp_sides = [s for s in item.needed_sides() if s != side]
        perp_has_agent = any(len(item.claimed_slots[s]) > 0 for s in perp_sides)
        perp_seeking_nearby = any(
            nb.push_slot is not None
            and nb.push_slot.get("item") is item
            and nb.push_slot.get("side") in perp_sides
            and nb._task == "seek_push_slot"
            for nb in self.neighbours
        )

        if not perp_has_agent:
            # Nobody going for diagonal — sprint, solo axis is the only option
            speed = AGENT_SPEED * 2.0
        elif perp_seeking_nearby:
            # A diagonal agent is nearby and getting into position — yield, let them anchor
            speed = AGENT_SPEED * 0.7
        else:
            # Diagonal agents are claimed but not close yet — normal pace
            speed = AGENT_SPEED

        arrived = self._move_toward(tx, ty, speed)

        if arrived:
            self._task       = "waiting_for_co_pusher"
            self._wait_timer = 0.0
            if self._side_ready(item, side):
                self._transition_side_to_pushing(item, side)

    def _side_ready(self, item, side):
        """True when this side's slots are all claimed, all holders are in task state,
        and all OTHER holders are physically ≤2px from their slot position."""
        slots = item.claimed_slots[side]
        if len(slots) < item.agents_per_side:
            return False
        for i, a in enumerate(slots):
            if a is self:
                continue
            if a._task not in ("waiting_for_co_pusher", "pushing"):
                return False
            tx, ty = item.slot_position(side, i)
            if math.hypot(a.x - tx, a.y - ty) > 2:
                return False
        return True

    def _do_waiting_for_co_pusher(self, dt):
        if self.push_slot is None:
            self._reset()
            return
        item = self.push_slot["item"]
        side = self.push_slot["side"]
        self._wait_timer += dt

        if item.delivered or (item.stored and item.phase == "to_store"):
            self._release_and_reset()
            return

        # Track slot position — item may already be moving on the other axis
        try:
            idx = item.claimed_slots[side].index(self)
        except ValueError:
            self._release_and_reset()
            return
        self.push_slot["idx"] = idx
        tx, ty = item.slot_position(side, idx)
        dist_to_slot = math.hypot(self.x - tx, self.y - ty)

        # If displaced too far from slot (item moved us), go back into seek
        if dist_to_slot > 2:
            self._task = "seek_push_slot"
            return

        if self._side_ready(item, side):
            self._transition_side_to_pushing(item, side)

    def _transition_side_to_pushing(self, item, side):
        """Start pushing for this side only. Other sides may still be filling."""
        if item.phase == "to_goal" and item.stored:
            item.stored = False
        self._task       = "pushing"
        self._wait_timer = 0.0
        for agent in item.claimed_slots[side]:
            if agent is not self and agent._task == "waiting_for_co_pusher":
                agent._task       = "pushing"
                agent._wait_timer = 0.0

    def _should_push(self, item, side):
        """Per-axis overshoot guard — stop pushing if item has passed destination on this axis."""
        if item.destination is None:
            return False
        dx = item.destination[0] - item.x
        dy = item.destination[1] - item.y
        if side == "LEFT":   return dx >  5    # need rightward motion
        if side == "RIGHT":  return dx < -5    # need leftward motion
        if side == "TOP":    return dy >  5    # need downward motion
        if side == "BOTTOM": return dy < -5    # need upward motion
        return False

    def _do_pushing(self, dt, zones, captains):
        if self.push_slot is None:
            self._reset()
            return
        item = self.push_slot["item"]
        side = self.push_slot["side"]

        # Another agent already finalized this item
        if item.delivered or (item.stored and item.phase == "to_store"):
            self._release_and_reset()
            return

        # Check arrival BEFORE applying any force this frame
        if item.destination is not None:
            dist = math.hypot(item.x - item.destination[0],
                              item.y - item.destination[1])
            if dist < DEST_THRESHOLD:
                self._finalize_delivery(item, zones, captains)
                return

        # Only push if item still needs to move in our direction (overshoot guard)
        if self._should_push(item, side):
            item.apply_push(side, PUSH_FORCE)
        elif side not in item.needed_sides():
            # This axis is genuinely no longer needed — break off and free up
            self._release_and_reset()
            return

        # Follow item — stay on slot position as it moves
        try:
            idx = item.claimed_slots[side].index(self)
        except ValueError:
            self._release_and_reset()
            return
        self.push_slot["idx"] = idx
        tx, ty = item.slot_position(side, idx)
        self._move_toward(tx, ty, AGENT_SPEED * 1.4)

    def _finalize_delivery(self, item, zones, captains):
        """Called by the first pusher to detect arrival. Resets all slot holders."""
        # Guard: another agent may have already finalized this frame
        if item.destination is None or item.delivered or (item.stored and item.phase == "to_store"):
            self._release_and_reset()
            return

        # Zero accumulator immediately so physics_update can't move item this frame
        item._push_fx = 0.0
        item._push_fy = 0.0
        item.vx       = 0.0
        item.vy       = 0.0
        item.destination = None   # acts as a lock against double-finalization

        if item.phase == "to_store":
            item.stored = True
            sx, sy = zones["storing"].center()
            item.x = sx + random.uniform(-40, 40)
            item.y = sy + random.uniform(-30, 30)
            for cap in captains:
                cap.sync_memory(item)
        else:  # to_goal
            item.delivered = True
            item.stored    = False

        # Release all slot holders
        for side_list in item.claimed_slots.values():
            for agent in list(side_list):
                if agent is not self:
                    agent.push_slot  = None
                    agent._task      = None
                    agent.going_for  = None
                    agent._wait_timer = 0.0
                    agent._pick_random_target()
        item.claimed_slots = {"LEFT": [], "RIGHT": [], "TOP": [], "BOTTOM": []}

        self.push_slot   = None
        self._task       = None
        self.going_for   = None
        self._wait_timer = 0.0
        self._pick_random_target()

    def _release_and_reset(self):
        if self.push_slot:
            self.push_slot["item"].release_slot(self)
            self.push_slot = None
        self._task       = None
        self.going_for   = None
        self._wait_timer = 0.0
        self._pick_random_target()

    def _reset(self):
        self._task      = None
        self.going_for  = None
        self._pick_random_target()

    # ------------------------------------------------------------------
    # Collision
    # ------------------------------------------------------------------

    def _resolve_item_collisions(self, items):
        """Hard separation from items we are not pushing."""
        half = AGENT_SIZE // 2
        for item in items:
            if item.stored or item.delivered:
                continue
            if self.push_slot and self.push_slot["item"] is item:
                continue  # in contact intentionally
            rect = item.collision_rect
            ax1, ay1 = self.x - half, self.y - half
            ax2, ay2 = self.x + half, self.y + half
            if ax2 > rect.left and ax1 < rect.right and ay2 > rect.top and ay1 < rect.bottom:
                ox = min(ax2 - rect.left, rect.right - ax1)
                oy = min(ay2 - rect.top, rect.bottom - ay1)
                if ox < oy:
                    self.x += ox if self.x > rect.centerx else -ox
                else:
                    self.y += oy if self.y > rect.centery else -oy

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, surface, font):
        cx, cy = int(self.x), int(self.y)
        half   = AGENT_SIZE // 2

        if self.busy:
            color = COLORS["agent_busy"]
        elif self._task == "pushing":
            color = COLORS["agent_push"]
        elif self._task in ("waiting_for_co_pusher",):
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
        elif self._task:
            task_label = {
                "seek_push_slot":      "seek",
                "waiting_for_co_pusher": "wait",
                "pushing":             "push",
            }.get(self._task, self._task)
            s = font.render(task_label, True, COLORS["text_dim"])
            surface.blit(s, (cx - s.get_width() // 2, cy + half + 2))

    def draw_comm_links(self, surface):
        for nb in self.neighbours:
            if nb.agent_id > self.agent_id:
                pygame.draw.line(surface, COLORS["comm_link"],
                                 (int(self.x), int(self.y)),
                                 (int(nb.x),   int(nb.y)), 1)

    def draw_push_links(self, surface):
        """Draw lines between agents pushing the same item."""
        if self.push_slot is None:
            return
        item = self.push_slot["item"]
        for side_list in item.claimed_slots.values():
            for agent in side_list:
                if agent is not self and agent.agent_id > self.agent_id:
                    pygame.draw.line(surface, COLORS["push_link"],
                                     (int(self.x), int(self.y)),
                                     (int(agent.x), int(agent.y)), 2)
        # Also draw line from agent to item center
        pygame.draw.line(surface, (*COLORS["push_link"], 120),
                         (int(self.x), int(self.y)),
                         (int(item.x), int(item.y)), 1)
