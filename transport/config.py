# Cooperative multi-robot object transport -- logic-only prototype (v3).
# Grid layout now mirrors simulation/grid.py's pattern directly: a
# cols x rows Cell grid, base-filled with one type ("walkway"), with named
# regions stamped on top from config-defined (col, row, w, h) rectangles --
# same idea as farm's WATER_REGION/PLANT_REGION/HARVEST_BOX.
#
# Robot movement itself still stays continuous/free-form, not grid-locked --
# the grid exists for obstacle bookkeeping (shelves are real hard barriers
# now, not just decoration) and for real BFS routing to a pickup slot, the
# same way farm's Grid.shortest_path routes an agent to a plant cell.

CANVAS_WIDTH  = 900
CANVAS_HEIGHT = 600
FPS           = 60

GRID_CELL = 30
GRID_COLS = CANVAS_WIDTH // GRID_CELL    # 30
GRID_ROWS = CANVAS_HEIGHT // GRID_CELL   # 20

# Shelf strips (col, row, w, h) -- 3 horizontal + 1 vertical, matching the
# reference layout. Each is SLOT_SPAN cells wide, evenly divisible into
# object segments -- one segment can hold one object at a time, and only
# that segment's cells unblock while it does (see warehouse.ShelfSlot).
SLOT_SPAN = 3
SHELF_REGIONS = [
    (3, 4,  12, 1),    # horizontal shelf 1
    (3, 9,  12, 1),    # horizontal shelf 2
    (3, 14, 12, 1),    # horizontal shelf 3
    (18, 3, 1, 12),    # vertical shelf
]
DROPOFF_REGION  = (22, 2,  5, 9)   # green -- delivery destination
STAGING_REGION  = (22, 13, 5, 5)   # orange -- robots start/idle here, not a delivery target

# Robots
NUM_ROBOTS             = 8
ROBOT_RADIUS           = 8
ROBOT_SPEED            = 90.0   # px/sec, solo travel while recruiting
ROBOT_ARRIVE_TOLERANCE = 4.0    # px -- how close counts as "actually in position"

# Objects -- footprint comes from whichever shelf slot they spawn in (see
# warehouse.ShelfSlot), not a random size. robots_needed still varies
# independently (stand-in for "how heavy/awkward this particular item is").
MIN_ROBOTS_PER_OBJECT = 2
MAX_ROBOTS_PER_OBJECT = 4

MAX_CONCURRENT_OBJECTS = 3
RESTOCK_INTERVAL_SEC   = 4.0   # how often an empty shelf slot may get a new object

DROPOFF_HOLD_SEC = 1.5   # visual pause once centered in the dropoff zone before delivery

# How close the group needs to get to a routed waypoint before advancing to
# the next one -- generous, since the object itself can be up to 90px long,
# it doesn't need pinpoint precision to "pass" a waypoint.
ROUTE_WAYPOINT_TOLERANCE = 20.0

# Negotiation (transport-phase local force weights) -- first-pass
# magnitudes, expect retuning once this is watched live. Parked for now
# (no rotation support yet) but left wired in, not ripped out.
GOAL_WEIGHT            = 1.0
OBSTACLE_WEIGHT        = 2.5
OBSTACLE_SENSE_RADIUS  = 60.0   # px, per-robot local sensing range
TRANSPORT_SPEED        = 55.0   # px/sec -- slower than solo travel, a loaded group moves cautiously

COLORS = {
    "background": (24, 26, 30),
    "hud_text":   (200, 200, 200),

    "wall":          (50, 50, 56),
    "walkway":       (24, 26, 30),    # same as background -- open floor
    "shelf_blocked": (120, 90, 40),   # empty/inactive shelf segment
    "shelf_open":    (200, 160, 70),  # segment currently hosting a pickup-able object
    "dropoff":       (40, 130, 80),
    "staging":       (200, 120, 40),

    "object_open":        (110, 110, 120),
    "object_approaching": (230, 200, 60),
    "object_anchored":    (70, 200, 100),
    "slot_open":          (180, 180, 190),
    "slot_filled":        (70, 200, 100),

    "robot_idle":     (90, 140, 230),
    "robot_moving":   (240, 150, 60),
    "robot_waiting":  (190, 90, 220),
    "robot_anchored": (70, 200, 100),
}
