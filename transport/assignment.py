"""Deterministic nearest-robot-to-nearest-open-slot assignment.

Placeholder for a future RL decision layer -- the point of this pass is to
get position-generation, routed navigation, and the readiness gate proven
and visually verified end-to-end first, on a decision rule simple enough to
reason about by eye.
"""


def assign_idle_robots(robots, objects, warehouse):
    idle_robots = [r for r in robots if r.state == "idle"]
    if not idle_robots:
        return

    for obj in objects:
        if obj.state != "open":
            continue
        for slot_idx in obj.open_slot_indices():
            if not idle_robots:
                break
            slot_pos = obj.slots[slot_idx]
            nearest = min(
                idle_robots,
                key=lambda r: (r.x - slot_pos[0]) ** 2 + (r.y - slot_pos[1]) ** 2,
            )
            path = warehouse.route(nearest.x, nearest.y, slot_pos[0], slot_pos[1])
            if not path:
                # Unreachable this tick -- shouldn't normally happen since a
                # slot's cells unblock the moment it holds an object, but
                # leave the robot idle and let assignment retry next tick
                # rather than assuming it can still reach the target.
                continue
            nearest.state = "moving"
            nearest.path = path
            nearest.assigned_object = obj
            nearest.assigned_slot = slot_idx
            obj.slot_occupants[slot_idx] = nearest
            idle_robots.remove(nearest)

        if all(occ is not None for occ in obj.slot_occupants):
            obj.state = "approaching"
