import math

from config import ROBOT_ARRIVE_TOLERANCE, ROBOT_SPEED


class Robot:
    def __init__(self, robot_id, x, y):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        # idle -> moving -> waiting -> anchored -> (released back to idle)
        self.state = "idle"
        # Waypoints to walk in order (world coords), not a single straight-
        # line target -- shelves are real obstacles now, so getting to a
        # pickup slot means following a routed path (warehouse.route),
        # ported from farm's BFS-over-cells idea, not beelining through walls.
        self.path = []
        self.assigned_object = None
        self.assigned_slot = None

    def step(self, dt):
        if self.state != "moving" or not self.path:
            return
        tx, ty = self.path[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist <= ROBOT_ARRIVE_TOLERANCE:
            self.x, self.y = tx, ty
            self.path.pop(0)
            if not self.path:
                self.state = "waiting"
            return
        step_dist = ROBOT_SPEED * dt
        if step_dist >= dist:
            self.x, self.y = tx, ty
            self.path.pop(0)
            if not self.path:
                self.state = "waiting"
        else:
            self.x += dx / dist * step_dist
            self.y += dy / dist * step_dist

    def release(self):
        self.state = "idle"
        self.path = []
        self.assigned_object = None
        self.assigned_slot = None


class TransportObject:
    def __init__(self, obj_id, cx, cy, width, height, robots_needed, spawn_tick):
        self.obj_id = obj_id
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height
        self.robots_needed = robots_needed
        # Offsets relative to center -- fixed for the object's lifetime, used
        # both as recruitment targets (cx/cy + offset, while the object still
        # sits still) and to keep rigid formation once it's moving (sim.py
        # re-derives each robot's live position from these every tick).
        self.slot_offsets = _underneath_slots(width, height, robots_needed)
        self.slots = [(cx + dx, cy + dy) for dx, dy in self.slot_offsets]
        self.slot_occupants = [None] * robots_needed   # Robot or None, by slot index
        # open -> approaching -> anchored -> delivered
        self.state = "open"
        self.anchor_timer = 0.0
        self.slot = None   # ShelfSlot this came from -- set by sim.py, cleared (reblocked) on pickup
        # BFS waypoints around obstacles toward the dropoff zone, computed
        # once at anchor time -- negotiation.py's local force-voting still
        # does the actual moment-to-moment movement/obstacle-avoidance, this
        # just gives it a near-term reachable target instead of a straight
        # line it can get stuck butting against (e.g. a shelf with no gap).
        self.route = []

        # Tick timestamps for baseline/eval measurement (eval_baseline.py) --
        # not used by the sim logic itself, purely observational.
        self.spawn_tick    = spawn_tick
        self.filled_tick   = None   # last slot claimed (state -> approaching)
        self.anchored_tick = None   # readiness gate passed (state -> anchored)

    def bounds(self, padding=0.0):
        x0 = self.cx - self.width / 2 - padding
        y0 = self.cy - self.height / 2 - padding
        x1 = self.cx + self.width / 2 + padding
        y1 = self.cy + self.height / 2 + padding
        return x0, y0, x1, y1

    def open_slot_indices(self):
        return [i for i, occ in enumerate(self.slot_occupants) if occ is None]

    def is_fully_ready(self):
        return all(
            occ is not None and occ.state == "waiting"
            for occ in self.slot_occupants
        )


def _underneath_slots(width, height, n):
    """n interior points spread under the object's footprint -- Kiva-style
    lift points, not perimeter attachment. Objects here are shelf segments
    (SLOT_SPAN cells long, one cell thick), so there's no room for a square
    lattice -- single file along the long axis is the only shape that fits
    without robots overlapping. Returned as (dx, dy) offsets from the
    object's center, not absolute positions."""
    if width >= height:
        cols, rows = n, 1
    else:
        cols, rows = 1, n
    inset_x = width / (cols + 1)
    inset_y = height / (rows + 1)
    points = []
    for i in range(n):
        col, row = i % cols, i // cols
        dx = -width / 2 + inset_x * (col + 1)
        dy = -height / 2 + inset_y * (row + 1)
        points.append((dx, dy))
    return points
