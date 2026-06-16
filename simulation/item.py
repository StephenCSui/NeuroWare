import math
import pygame
from config import ITEM_TYPES, COLORS, PUSH_SLOT_OFFSET, SLOT_SPREAD, ITEM_DAMPING, PUSH_SPEED_MAX


class Item:
    _next_id = 0

    def __init__(self, item_type, x, y):
        self.item_id         = Item._next_id
        Item._next_id       += 1
        self.item_type       = item_type
        self.x               = float(x)
        self.y               = float(y)
        self.agents_per_side = ITEM_TYPES[item_type]["agents_per_side"]

        # Push slot assignments per side — each list holds up to agents_per_side agent refs
        self.claimed_slots   = {"LEFT": [], "RIGHT": [], "TOP": [], "BOTTOM": []}

        # Physics
        self.vx              = 0.0
        self.vy              = 0.0
        self._push_fx        = 0.0   # force accumulated this frame
        self._push_fy        = 0.0

        # Task state — set by captain when item is pinged
        self.destination     = None  # (x, y) target
        self.phase           = None  # "to_store" | "to_goal"
        self.stored          = False
        self.delivered       = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def color(self):
        return ITEM_TYPES[self.item_type]["color"]

    @property
    def label(self):
        return f"{ITEM_TYPES[self.item_type]['label']}{self.item_id}"

    @property
    def size(self):
        return ITEM_TYPES[self.item_type]["size"]

    @property
    def collision_rect(self):
        half = self.size // 2
        return pygame.Rect(int(self.x) - half, int(self.y) - half, self.size, self.size)

    # ------------------------------------------------------------------
    # Slot management
    # ------------------------------------------------------------------

    def needed_sides(self):
        """Which push sides are required based on current destination direction."""
        if self.destination is None:
            return []
        dx = self.destination[0] - self.x
        dy = self.destination[1] - self.y
        sides = []
        if abs(dx) > 12:
            sides.append("LEFT" if dx > 0 else "RIGHT")
        if abs(dy) > 12:
            sides.append("TOP" if dy > 0 else "BOTTOM")
        return sides

    def slot_position(self, side, idx):
        """World position where agent at slot (side, idx) should stand."""
        half   = self.size // 2 + PUSH_SLOT_OFFSET
        spread = (idx - (self.agents_per_side - 1) / 2.0) * SLOT_SPREAD
        if side == "LEFT":
            return (self.x - half, self.y + spread)
        elif side == "RIGHT":
            return (self.x + half, self.y + spread)
        elif side == "TOP":
            return (self.x + spread, self.y - half)
        else:  # BOTTOM
            return (self.x + spread, self.y + half)

    def claim_slot(self, agent, side):
        """Try to claim a slot on this side. Returns True if successful."""
        slots = self.claimed_slots[side]
        if agent in slots:
            return True
        if len(slots) < self.agents_per_side:
            slots.append(agent)
            return True
        return False

    def release_slot(self, agent):
        """Remove agent from whichever side they hold."""
        for side_list in self.claimed_slots.values():
            if agent in side_list:
                side_list.remove(agent)
                return

    def all_needed_slots_filled(self):
        """True when every required side has a full complement of agents."""
        needed = self.needed_sides()
        if not needed:
            return False
        return all(len(self.claimed_slots[s]) >= self.agents_per_side for s in needed)

    def is_available(self):
        """True if the item has open slots an agent can claim."""
        if self.delivered:
            return False
        if self.destination is None:
            return False
        if self.stored and self.phase != "to_goal":
            return False
        needed = self.needed_sides()
        if not needed:
            return False
        return any(len(self.claimed_slots[s]) < self.agents_per_side for s in needed)

    # ------------------------------------------------------------------
    # Physics
    # ------------------------------------------------------------------

    def apply_push(self, side, force):
        """Accumulate push force from an agent this frame."""
        if side == "LEFT":
            self._push_fx += force
        elif side == "RIGHT":
            self._push_fx -= force
        elif side == "TOP":
            self._push_fy += force
        elif side == "BOTTOM":
            self._push_fy -= force

    def physics_update(self):
        """Integrate accumulated forces into position. Call once per frame."""
        if self._push_fx != 0.0 or self._push_fy != 0.0:
            self.vx = self._push_fx
            self.vy = self._push_fy
            speed = math.hypot(self.vx, self.vy)
            if speed > PUSH_SPEED_MAX:
                scale = PUSH_SPEED_MAX / speed
                self.vx *= scale
                self.vy *= scale
        else:
            self.vx *= ITEM_DAMPING
            self.vy *= ITEM_DAMPING
            if abs(self.vx) < 0.05:
                self.vx = 0.0
            if abs(self.vy) < 0.05:
                self.vy = 0.0

        self.x += self.vx
        self.y += self.vy

        # Keep item within window bounds
        half    = self.size // 2
        from config import WINDOW_WIDTH, WINDOW_HEIGHT
        if self.x - half < 0:
            self.x  = half;  self.vx = 0.0
        elif self.x + half > WINDOW_WIDTH:
            self.x  = WINDOW_WIDTH - half;  self.vx = 0.0
        if self.y - half < 0:
            self.y  = half;  self.vy = 0.0
        elif self.y + half > WINDOW_HEIGHT:
            self.y  = WINDOW_HEIGHT - half;  self.vy = 0.0

        # Reset accumulator for next frame
        self._push_fx = 0.0
        self._push_fy = 0.0

    def resolve_collision(self, other):
        """AABB push-apart between two items."""
        r1 = self.collision_rect
        r2 = other.collision_rect
        if not r1.colliderect(r2):
            return
        ox = min(r1.right, r2.right) - max(r1.left, r2.left)
        oy = min(r1.bottom, r2.bottom) - max(r1.top, r2.top)
        if ox < oy:
            if r1.centerx < r2.centerx:
                self.x  -= ox / 2
                other.x += ox / 2
            else:
                self.x  += ox / 2
                other.x -= ox / 2
        else:
            if r1.centery < r2.centery:
                self.y  -= oy / 2
                other.y += oy / 2
            else:
                self.y  += oy / 2
                other.y -= oy / 2

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, surface, font):
        cx, cy = int(self.x), int(self.y)
        sz     = self.size
        half   = sz // 2

        pygame.draw.rect(surface, self.color,
                         (cx - half, cy - half, sz, sz), border_radius=3)
        # Border thickness indicates agents_per_side
        pygame.draw.rect(surface, (255, 255, 255),
                         (cx - half, cy - half, sz, sz),
                         self.agents_per_side, border_radius=3)

        lbl = font.render(self.label, True, COLORS["text"])
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy - lbl.get_height() // 2))

        # Small indicator below label showing agents_per_side
        sub = font.render(f"x{self.agents_per_side}", True, COLORS["text_dim"])
        surface.blit(sub, (cx - sub.get_width() // 2, cy + half + 2))

        # Green dots on sides that have claimed agents
        side_positions = {
            "LEFT":   (cx - half - 8, cy),
            "RIGHT":  (cx + half + 4, cy),
            "TOP":    (cx, cy - half - 8),
            "BOTTOM": (cx, cy + half + 4),
        }
        for side, pos in side_positions.items():
            claimed = len(self.claimed_slots[side])
            if claimed > 0:
                pygame.draw.circle(surface, (80, 220, 80), pos, 5)
                cnt = font.render(str(claimed), True, (20, 20, 20))
                surface.blit(cnt, (pos[0] - cnt.get_width() // 2,
                                   pos[1] - cnt.get_height() // 2))
