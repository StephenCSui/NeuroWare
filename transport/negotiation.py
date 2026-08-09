"""Per-robot local force computation for the group-transport phase --
deliberately decentralized: each robot senses nearby obstacles from its own
position and casts its own vote for a direction; the group's actual motion
is the combination of those independent local votes, not a single
pre-planned route through the corridor computed once for the whole group.
This is the "ants, not a foreman" mechanic -- a robot pinned close to a wall
contributes a stronger correction than one with open space around it, and
the squeeze-through-the-corridor behavior has to emerge from that, not be
told where the gap is.

No cohesion term here: the group is kept in rigid formation by sim.py
snapping every robot to object.cx/cy + its slot_offset after each tick, so
there's never real drift for a cohesion force to correct. A softer,
non-rigid version of this (robots allowed to actually drift and pull each
other back) is future work once this moves to real physics -- rigidity here
is enforced by code, not a real mechanical joint.
"""
import math

from config import GOAL_WEIGHT, OBSTACLE_SENSE_RADIUS, OBSTACLE_WEIGHT, TRANSPORT_SPEED


def propose_group_velocity(obj, carrying_robots, warehouse, goal_xy, other_objects=()):
    gx, gy = goal_xy
    goal_dir = _normalize((gx - obj.cx, gy - obj.cy))
    other_bounds = [o.bounds() for o in other_objects]

    votes = []
    for robot in carrying_robots:
        repulsion = (0.0, 0.0)
        for ox, oy in warehouse.obstacle_cells_near(robot.x, robot.y, OBSTACLE_SENSE_RADIUS):
            dx, dy = robot.x - ox, robot.y - oy
            dist = math.hypot(dx, dy) or 1e-6
            strength = max(0.0, (OBSTACLE_SENSE_RADIUS - dist) / OBSTACLE_SENSE_RADIUS)
            repulsion = (repulsion[0] + dx / dist * strength,
                         repulsion[1] + dy / dist * strength)

        # Other objects push back too, not just static walls/shelves --
        # otherwise two moving groups have no way to steer around each
        # other and can only ever hard-stop face to face.
        for rect in other_bounds:
            ox, oy = _closest_point_on_rect(robot.x, robot.y, rect)
            dx, dy = robot.x - ox, robot.y - oy
            dist = math.hypot(dx, dy) or 1e-6
            if dist >= OBSTACLE_SENSE_RADIUS:
                continue
            strength = max(0.0, (OBSTACLE_SENSE_RADIUS - dist) / OBSTACLE_SENSE_RADIUS)
            repulsion = (repulsion[0] + dx / dist * strength,
                         repulsion[1] + dy / dist * strength)

        vx = GOAL_WEIGHT * goal_dir[0] + OBSTACLE_WEIGHT * repulsion[0]
        vy = GOAL_WEIGHT * goal_dir[1] + OBSTACLE_WEIGHT * repulsion[1]
        votes.append((vx, vy))

    avg_vx = sum(v[0] for v in votes) / len(votes)
    avg_vy = sum(v[1] for v in votes) / len(votes)
    direction = _normalize((avg_vx, avg_vy))
    return direction[0] * TRANSPORT_SPEED, direction[1] * TRANSPORT_SPEED


def _closest_point_on_rect(px, py, rect):
    x0, y0, x1, y1 = rect
    return min(max(px, x0), x1), min(max(py, y0), y1)


def _normalize(vec):
    x, y = vec
    length = math.hypot(x, y)
    if length < 1e-6:
        return (0.0, 0.0)
    return (x / length, y / length)
