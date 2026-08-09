import math
import random

from assignment import assign_idle_robots
from config import (DROPOFF_HOLD_SEC, MAX_CONCURRENT_OBJECTS,
                     MAX_ROBOTS_PER_OBJECT, MIN_ROBOTS_PER_OBJECT, NUM_ROBOTS,
                     RESTOCK_INTERVAL_SEC, ROUTE_WAYPOINT_TOLERANCE)
from entities import Robot, TransportObject
from negotiation import propose_group_velocity
from warehouse import Warehouse


class Simulation:
    def __init__(self):
        self.warehouse = Warehouse()
        sx0, sy0, sx1, sy1 = self.warehouse.staging_bounds()
        self.robots = [
            Robot(i, random.uniform(sx0, sx1), random.uniform(sy0, sy1))
            for i in range(NUM_ROBOTS)
        ]
        self.objects = []
        self._next_obj_id = 0
        self._restock_timer = RESTOCK_INTERVAL_SEC   # restock one immediately
        self.tick_count = 0
        # (spawn_tick, filled_tick, anchored_tick, delivered_tick) per
        # delivered object -- read by eval_baseline.py, not used by sim logic.
        self.completed_records = []

    def tick(self, dt):
        self.tick_count += 1

        self._restock_timer += dt
        if self._restock_timer >= RESTOCK_INTERVAL_SEC:
            self._restock_timer = 0.0
            self._try_restock()

        assign_idle_robots(self.robots, self.objects, self.warehouse)

        for obj in self.objects:
            if obj.state == "approaching" and obj.filled_tick is None:
                obj.filled_tick = self.tick_count

        for robot in self.robots:
            robot.step(dt)   # no-op unless state == "moving"

        for obj in self.objects:
            if obj.state == "approaching" and obj.is_fully_ready():
                obj.state = "anchored"
                obj.anchored_tick = self.tick_count
                # Note: NOT clearing obj.slot here. The object is still
                # physically sitting on those cells at this instant -- if we
                # reblock them immediately, its very first move attempt
                # overlaps its own just-vacated cells and gets rejected as
                # "blocked", permanently trapping it. Reblocked lazily in
                # _advance_transport once it has actually moved clear.
                gx0, gy0, gx1, gy1 = self.warehouse.dropoff_bounds()
                goal_x, goal_y = (gx0 + gx1) / 2, (gy0 + gy1) / 2
                escape_x, escape_y = self.warehouse.escape_point(obj.slot)
                obj.route = [(escape_x, escape_y)] + self.warehouse.route_for_footprint(
                    escape_x, escape_y, goal_x, goal_y, obj.width, obj.height)
                for occ in obj.slot_occupants:
                    occ.state = "anchored"

        for obj in self.objects:
            if obj.state == "anchored":
                self._advance_transport(obj, dt)

        delivered = [o for o in self.objects if o.state == "delivered"]
        for obj in delivered:
            for occ in obj.slot_occupants:
                occ.release()
            self.completed_records.append(
                (obj.spawn_tick, obj.filled_tick, obj.anchored_tick, self.tick_count)
            )
            self.objects.remove(obj)

    def _advance_transport(self, obj, dt):
        """Group is locked onto the object -- move it toward the dropoff
        zone. Direction comes from negotiation.py's per-robot local votes,
        aimed at the next unreached waypoint of obj.route (a BFS route
        around obstacles computed once at anchor time) rather than the
        dropoff zone directly -- that route is what keeps local force-voting
        from getting stuck against something like the vertical shelf, which
        has no gap nearby for pure local sensing to discover on its own.
        This still isn't a rigid pre-planned path for the whole trip: local
        votes do the actual movement and immediate obstacle-avoidance,
        the route just keeps handing them a reachable near-term target."""
        dx0, dy0, dx1, dy1 = self.warehouse.dropoff_bounds()
        final_goal = ((dx0 + dx1) / 2, (dy0 + dy1) / 2)

        while obj.route and math.hypot(obj.route[0][0] - obj.cx, obj.route[0][1] - obj.cy) <= ROUTE_WAYPOINT_TOLERANCE:
            obj.route.pop(0)
        goal = obj.route[0] if obj.route else final_goal

        others = [o for o in self.objects if o is not obj]
        vx, vy = propose_group_velocity(obj, obj.slot_occupants, self.warehouse, goal, other_objects=others)
        cand_cx, cand_cy = obj.cx + vx * dt, obj.cy + vy * dt

        if not self._object_blocked_at(obj, cand_cx, cand_cy):
            obj.cx, obj.cy = cand_cx, cand_cy
        elif not self._object_blocked_at(obj, cand_cx, obj.cy):
            obj.cx = cand_cx
        elif not self._object_blocked_at(obj, obj.cx, cand_cy):
            obj.cy = cand_cy
        # else: fully blocked this tick, hold position.

        for slot_idx, robot in enumerate(obj.slot_occupants):
            dx, dy = obj.slot_offsets[slot_idx]
            robot.x = obj.cx + dx
            robot.y = obj.cy + dy

        if obj.slot is not None and not _overlaps(obj.bounds(), obj.slot.world_bounds()):
            obj.slot.clear()
            obj.slot = None

        if dx0 <= obj.cx <= dx1 and dy0 <= obj.cy <= dy1:
            obj.anchor_timer += dt
            if obj.anchor_timer >= DROPOFF_HOLD_SEC:
                obj.state = "delivered"
        else:
            obj.anchor_timer = 0.0

    def _object_blocked_at(self, obj, cx, cy):
        x0, y0 = cx - obj.width / 2, cy - obj.height / 2
        x1, y1 = cx + obj.width / 2, cy + obj.height / 2
        if self.warehouse.rect_overlaps_obstacle(x0, y0, x1, y1):
            return True
        # Also real objects, not just the warehouse grid -- otherwise two
        # objects (e.g. one still sitting in a shelf slot, or another mid-
        # transport) can freely overlap each other, which is exactly the
        # "using another object as an opening" cheat.
        candidate = (x0, y0, x1, y1)
        for other in self.objects:
            if other is obj:
                continue
            if _overlaps(candidate, other.bounds()):
                return True
        return False

    def _try_restock(self):
        if len(self.objects) >= MAX_CONCURRENT_OBJECTS:
            return
        empty_slots = [s for s in self.warehouse.slots if s.item is None]
        if not empty_slots:
            return

        slot = random.choice(empty_slots)
        cx, cy = slot.center()
        x0, y0, x1, y1 = slot.world_bounds()
        robots_needed = random.randint(MIN_ROBOTS_PER_OBJECT, MAX_ROBOTS_PER_OBJECT)

        obj = TransportObject(self._next_obj_id, cx, cy, x1 - x0, y1 - y0,
                               robots_needed, self.tick_count)
        obj.slot = slot
        slot.occupy(obj)
        self.objects.append(obj)
        self._next_obj_id += 1


def _overlaps(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return ax0 < bx1 and ax1 > bx0 and ay0 < by1 and ay1 > by0
