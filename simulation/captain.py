import pygame
from config import (CAPTAIN_RADIUS, CAPTAIN_PING_INTERVAL, COMM_RANGE,
                    COLORS, CAPTAIN_A_ZONE, CAPTAIN_B_ZONE, STORING_ZONE)

# Storing zone center — set as item destination for the "to_store" phase
_sz = STORING_ZONE
STORING_CENTER = (_sz[0] + _sz[2] // 2, _sz[1] + _sz[3] // 2)


class Captain:
    def __init__(self, captain_id, x, y, zone_rect, zone_color_key, other=None):
        self.captain_id      = captain_id
        self.x               = x
        self.y               = y
        self.zone_rect       = pygame.Rect(*zone_rect)
        self.zone_color_key  = zone_color_key
        self.other           = other

        self.memory          = {}   # item_id → item
        self.active_pings    = {}   # item_id → item  (needs push to store)
        self.goal_pings      = {}   # item_id → (item, goal_center)
        self.pending_syncs   = []   # [(delay_remaining, item)]

        self.ping_timer      = 0.0
        self.ping_radius     = 0.0
        self.ping_active     = False

        self.SYNC_DELAY      = 2.0

    def link(self, other):
        self.other = other

    def in_zone(self, x, y):
        return self.zone_rect.collidepoint(x, y)

    def update(self, dt, items):
        for iid in list(self.active_pings.keys()):
            it = self.active_pings[iid]
            if it.stored or it.delivered or not it.is_available():
                del self.active_pings[iid]

        for iid in list(self.goal_pings.keys()):
            it, _ = self.goal_pings[iid]
            if it.delivered or it.phase != "to_goal":
                del self.goal_pings[iid]

        still_pending = []
        for delay, item in self.pending_syncs:
            delay -= dt
            if delay <= 0:
                self._activate_sync(item)
            else:
                still_pending.append((delay, item))
        self.pending_syncs = still_pending

        self.ping_timer += dt
        if self.ping_timer >= CAPTAIN_PING_INTERVAL:
            self.ping_timer = 0.0
            self._scan_for_items(items)

        if self.ping_active:
            self.ping_radius += 120 * dt
            if self.ping_radius > COMM_RANGE * 1.5:
                self.ping_active = False
                self.ping_radius = 0.0

    def _scan_for_items(self, items):
        found = False
        for item in items:
            if item.stored or item.delivered:
                continue
            if item.phase == "to_goal":
                # Already assigned for goal delivery — never re-claim
                continue
            if not self.in_zone(item.x, item.y):
                continue
            if item.item_id not in self.active_pings:
                item.destination = STORING_CENTER
                item.phase       = "to_store"
                self.active_pings[item.item_id] = item
                self.memory[item.item_id]        = item
                if self.other:
                    self.other.sync_memory(item)
            found = True
        if found:
            self.ping_active = True
            self.ping_radius = 0.0

    def sync_memory(self, item):
        self.memory[item.item_id] = item
        if not item.stored and not item.delivered and item.item_id not in self.active_pings:
            already_queued = any(i.item_id == item.item_id for _, i in self.pending_syncs)
            if not already_queued:
                self.pending_syncs.append((self.SYNC_DELAY, item))

    def _activate_sync(self, item):
        if (not item.stored and not item.delivered
                and item.phase != "to_goal"
                and item.item_id not in self.active_pings):
            self.active_pings[item.item_id] = item
            self.ping_active = True
            self.ping_radius = 0.0

    def receive_goal_request(self, item_type, goal_center):
        if self._try_dispatch_goal(item_type, goal_center):
            return True
        if self.other and self.other._try_dispatch_goal(item_type, goal_center):
            return True
        return False

    def _try_dispatch_goal(self, item_type, goal_center):
        for iid, item in self.memory.items():
            if (item.item_type == item_type
                    and item.stored
                    and not item.delivered
                    and not any(item.claimed_slots.values())
                    and iid not in self.goal_pings
                    and self.in_zone(item.x, item.y)):
                # Set destination and phase on item — agents read these directly
                item.destination = goal_center
                item.phase       = "to_goal"
                self.goal_pings[iid] = (item, goal_center)
                self.ping_active = True
                self.ping_radius = 0.0
                if self.other:
                    self.other.goal_pings[iid] = (item, goal_center)
                return True
        return False

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw_zone(self, surface):
        overlay = pygame.Surface(
            (self.zone_rect.width, self.zone_rect.height), pygame.SRCALPHA)
        r, g, b = COLORS[self.zone_color_key]
        overlay.fill((r, g, b, 25))
        surface.blit(overlay, (self.zone_rect.x, self.zone_rect.y))
        pygame.draw.rect(surface, COLORS[self.zone_color_key], self.zone_rect, 2)

    def draw(self, surface, font):
        color = COLORS[self.zone_color_key]
        pygame.draw.circle(surface, color, (self.x, self.y), CAPTAIN_RADIUS)
        pygame.draw.circle(surface, (255, 255, 255), (self.x, self.y),
                           CAPTAIN_RADIUS, 2)

        lbl = font.render(f"C{self.captain_id}", True, COLORS["text"])
        surface.blit(lbl, (self.x - lbl.get_width() // 2,
                            self.y - CAPTAIN_RADIUS - 16))

        ping_count = len(self.active_pings) + len(self.goal_pings)
        mem_lbl = font.render(
            f"mem:{len(self.memory)}  ping:{ping_count}",
            True, COLORS["text_dim"])
        surface.blit(mem_lbl, (self.x - mem_lbl.get_width() // 2,
                                self.y + CAPTAIN_RADIUS + 4))

        if self.ping_active:
            alpha = max(0, int(200 * (1 - self.ping_radius / (COMM_RANGE * 1.5))))
            r = int(self.ping_radius)
            ping_surf = pygame.Surface((r * 2 + 2, r * 2 + 2), pygame.SRCALPHA)
            pygame.draw.circle(ping_surf, (*COLORS["ping"], alpha),
                               (r + 1, r + 1), r, 2)
            surface.blit(ping_surf, (self.x - r - 1, self.y - r - 1))

    def draw_captain_link(self, surface, other):
        syncing = bool(self.pending_syncs or other.pending_syncs)
        color   = (255, 220, 80) if syncing else COLORS["captain_link"]
        width   = 2 if syncing else 1
        pygame.draw.line(surface, color, (self.x, self.y), (other.x, other.y), width)
        font = pygame.font.SysFont("monospace", 11)
        mid  = ((self.x + other.x) // 2, (self.y + other.y) // 2)
        label = "syncing..." if syncing else "sync"
        lbl  = font.render(label, True, color)
        surface.blit(lbl, (mid[0] - lbl.get_width() // 2,
                            mid[1] - lbl.get_height() // 2))
