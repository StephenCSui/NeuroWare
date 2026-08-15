"""Live 2D occupancy grid + A* path planner for the two-robot transport
demo (see two_robot_pickup_demo.py). Deliberately independent of any one
robot/camera/simulation API -- update_from_camera() only needs a camera-like
object with get_world_pose(camera_axes="world") and a raw depth array handed
to it by the caller, so this module has no circular dependency on the main
script and stays testable on its own.

Shared between both robots: one grid instance is updated by both cameras,
since they occupy the same physical space and each robot benefits from what
the other has already seen.

Deliberately NOT seeded from known shelf geometry -- the grid only knows what
a camera has actually observed. This is what makes it live: shelf structure,
spawn_obstacle() cubes, and anything moved by hand during a live session are
all reflected the same way, purely from sensing, nothing baked in. An
obstacle that moves away has its old cells re-marked FREE the next time a
ray passes cleanly through them -- no decay/timestamp bookkeeping needed,
just re-scanning.
"""
import heapq
import math

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2

# World-space bounding box this grid covers -- generous margin around both
# shelves and the corridor between them, derived from real positions on
# record (isaac_environment.md payload table, DELIVERY_Y=3.25) and this
# session's live obstacle-avoidance traces (excursions out to x~1.96,
# y~3.26). This is map EXTENT/RESOLUTION -- fixed configuration, not a
# movement behavior like the old fixed turn-angle/hop-distance constants
# it's replacing.
#
# X_MIN was -2.0 -- confirmed live to be too tight: Robot2's actual spawn
# chassis position is x=-2.064, already outside that boundary before its
# corner cameras (offset further out still) are even considered. Widened
# with real margin so both robots' full operating range, corners included,
# stays genuinely inside the mapped area.
X_MIN, X_MAX = -2.5, 3.0
Y_MIN, Y_MAX = -2.0, 4.5
CELL_SIZE = 0.1  # meters
GRID_W = int(round((X_MAX - X_MIN) / CELL_SIZE))
GRID_H = int(round((Y_MAX - Y_MIN) / CELL_SIZE))

MAX_RANGE = 3.0  # meters -- matches DEPTH_VIS_MAX, the established useful
                  # range for this robot's depth camera.

# Chassis half-extents, from base.usda (chassis_visual 0.3x0.15x0.04 box).
# Inflation uses the ROTATION-SWEPT radius (half-diagonal), not half-width
# alone -- a rotating rectangle sweeps out a circle of that radius, and
# every waypoint the planner stops at requires an in-place turn to face
# the next one. Confirmed live this was the actual bug behind a whole
# session of "clear to be here but not clear to rotate here" failures: a
# translation-only inflation let the planner route to spots that were
# fine to pass through but not to stop and turn at. Inflating by the full
# swept radius everywhere means any successfully-planned path is safe to
# rotate at every waypoint by construction -- no separate rotation check
# needed anywhere.
CHASSIS_HALF_X = 0.15
CHASSIS_HALF_Y = 0.075


def rotation_swept_radius(cargo_half_x=0.0, cargo_half_y=0.0):
    """Inflation radius for planning: the chassis's own rotation-swept
    circle, grown by whatever's currently being carried. cargo_half_x/y
    should be the carried object's own real half-extents (queried from
    the scene, not guessed -- see OBJECT_HALF_EXTENTS in
    two_robot_pickup_demo.py) so a tiny object doesn't get the same
    clearance requirement as Cube_01. Summed (not maxed) with the
    chassis's own radius as a deliberately conservative bound, since the
    cargo's exact position on the plate relative to the chassis's
    rotation center isn't modeled precisely."""
    chassis_r = math.hypot(CHASSIS_HALF_X, CHASSIS_HALF_Y)
    cargo_r = math.hypot(cargo_half_x, cargo_half_y)
    return chassis_r + cargo_r

SCAN_ROW_HALF_BAND = 1  # sample only rows within this many pixels of the
                          # exact vertical center. A wider band was tried
                          # first (a fraction of image height) but was a
                          # real bug: the ray-direction math below is
                          # purely horizontal (z=0), which is only true for
                          # the exact center row -- any row off-center
                          # corresponds to a real ray angled up/down, and
                          # treating its distance reading as if it were
                          # horizontal projects a floor hit (a real, close
                          # depth reading, but of the ground, not an
                          # obstacle) out to a phantom position meters away
                          # in the XY plane. Confirmed live: this produced
                          # a smooth arc of false OCCUPIED cells sweeping
                          # through genuinely empty corridor space as the
                          # robot rotated, present from the very first
                          # scan of a run, with no real geometry anywhere
                          # near it. A near-single-row band keeps every
                          # sampled ray close enough to genuinely
                          # horizontal that the existing math is valid.
SCAN_COLS = 32  # columns subsampled across the frame width -- cheap enough
                # to run every tick for both cameras.
HORIZONTAL_FOV_DEG = 90.0  # matches set_focal_length(10.5), established
                            # this session (~90deg horizontal FOV).


def _rotate_vector(quat_wxyz, v):
    """Standard quaternion-rotate-vector formula, generalized from the
    forward-vector-only version already established and verified in
    identify_obstacle() (two_robot_pickup_demo.py) -- same math, just for
    an arbitrary local direction instead of only local +X."""
    w, x, y, z = quat_wxyz
    qv = np.array([x, y, z])
    t = 2 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


class OccupancyGrid:
    def __init__(self):
        self.cells = np.full((GRID_H, GRID_W), UNKNOWN, dtype=np.int8)

    def world_to_cell(self, x, y):
        # math.floor, NOT int() -- int() truncates toward zero, which
        # silently maps any out-of-bounds negative coordinate into cell 0
        # (a VALID-looking cell) instead of a genuinely negative index that
        # in_bounds() would correctly reject. Confirmed live: Robot2's own
        # spawn position (x=-2.064) sits just outside X_MIN=-2.0, and with
        # int() truncation its cameras' rays were being silently computed
        # from cell 0 as if the robot were exactly at the boundary --
        # producing a whole arc of corrupted cells as a function of each
        # ray's angle, not a sensor problem at all.
        return math.floor((x - X_MIN) / CELL_SIZE), math.floor((y - Y_MIN) / CELL_SIZE)

    def cell_to_world(self, cx, cy):
        return X_MIN + (cx + 0.5) * CELL_SIZE, Y_MIN + (cy + 0.5) * CELL_SIZE

    def in_bounds(self, cx, cy):
        return 0 <= cx < GRID_W and 0 <= cy < GRID_H

    def mark_free(self, cx, cy):
        if self.in_bounds(cx, cy):
            self.cells[cy, cx] = FREE

    def mark_occupied(self, cx, cy):
        if self.in_bounds(cx, cy):
            self.cells[cy, cx] = OCCUPIED

    def _raymarch(self, x0, y0, x1, y1, hit):
        """Marks cells along the line from (x0,y0) to (x1,y1) FREE, and the
        end cell OCCUPIED if hit (a max-range ray with nothing detected
        just means that whole line is free, as far as this scan can tell --
        the end cell is left FREE too in that case)."""
        cx0, cy0 = self.world_to_cell(x0, y0)
        cx1, cy1 = self.world_to_cell(x1, y1)
        steps = max(abs(cx1 - cx0), abs(cy1 - cy0), 1)
        for i in range(steps + 1):
            t = i / steps
            cx = int(round(cx0 + (cx1 - cx0) * t))
            cy = int(round(cy0 + (cy1 - cy0) * t))
            if i == steps and hit:
                self.mark_occupied(cx, cy)
            else:
                self.mark_free(cx, cy)


def mark_detected_obstacle(grid, cam_pos, cam_quat, dist):
    """Directly marks the grid cell at a known real-time detection (e.g. a
    reactive close-range obstacle stop) as OCCUPIED, computed along the
    camera's own forward (local +X) direction at the given distance.

    Ground truth from an immediate sensor reading takes precedence over
    the passive per-tick scan (update_from_camera) -- confirmed live this
    session: at close range near an obstacle's corner/edge, a neighboring
    ray at a slightly different angle can land in the same grid cell and
    overwrite a real "occupied" hit with a "free" reading later in the
    same tick's scan, since the grid has no occupied-beats-free
    precedence rule. Writing a reactive detection straight in sidesteps
    that entirely, and is what lets the grid (global planning) actually
    learn from what the camera (close-range/immediate decision) just
    proved, instead of the two silently disagreeing forever -- previously
    a reactive block would immediately replan the exact same route, since
    the grid never recorded why it had been stopped."""
    px, py = float(cam_pos[0]), float(cam_pos[1])
    world_dir = _rotate_vector(cam_quat, np.array([1.0, 0.0, 0.0]))
    hx, hy = px + world_dir[0] * dist, py + world_dir[1] * dist
    cx, cy = grid.world_to_cell(hx, hy)
    grid.mark_occupied(cx, cy)


def update_from_camera(grid, cam, depth_array):
    """Ray-marches a horizontal band of the camera's current depth frame
    into the shared grid. depth_array is passed in by the caller (already
    has raw_depth_frame(cam) available) rather than fetched here, so this
    module never needs to import from the main script."""
    if depth_array is None:
        return
    depth_array = np.asarray(depth_array)
    h, w = depth_array.shape
    if h < 2 or w < 2:
        return
    pos, quat = cam.get_world_pose(camera_axes="world")
    px, py = float(pos[0]), float(pos[1])
    row_center = h // 2
    row_lo = max(0, row_center - SCAN_ROW_HALF_BAND)
    row_hi = min(h, row_center + SCAN_ROW_HALF_BAND + 1)
    half_fov = math.radians(HORIZONTAL_FOV_DEG) / 2.0
    cols = np.linspace(0, w - 1, SCAN_COLS).astype(int)
    for c in cols:
        angle = (c / (w - 1) - 0.5) * 2 * half_fov
        local_dir = np.array([math.cos(angle), math.sin(angle), 0.0])
        world_dir = _rotate_vector(quat, local_dir)
        col_vals = depth_array[row_lo:row_hi, c]
        finite = col_vals[np.isfinite(col_vals)]
        if finite.size:
            dist = min(float(finite.min()), MAX_RANGE)
            hit = dist < MAX_RANGE
        else:
            dist = MAX_RANGE
            hit = False
        hx = px + world_dir[0] * dist
        hy = py + world_dir[1] * dist
        grid._raymarch(px, py, hx, hy, hit)


def _neighbors(cx, cy):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield cx + dx, cy + dy, math.hypot(dx, dy)


def _inflated(cells, radius):
    """Grows OCCUPIED by `radius` (meters, converted to whole cells) in
    every direction so planning keeps real clearance automatically,
    instead of relying on a reactive distance threshold at drive time.
    `radius` should be a rotation-swept radius (see rotation_swept_radius)
    that already accounts for whatever's currently being carried -- not a
    single fixed constant, since what's safe to rotate through genuinely
    differs between carrying nothing and carrying Cube_01."""
    inflate_cells = max(1, int(math.ceil(radius / CELL_SIZE)))
    inflated = cells == OCCUPIED
    for _ in range(inflate_cells):
        grown = inflated.copy()
        grown[1:, :] |= inflated[:-1, :]
        grown[:-1, :] |= inflated[1:, :]
        grown[:, 1:] |= inflated[:, :-1]
        grown[:, :-1] |= inflated[:, 1:]
        inflated = grown
    return inflated


def _line_of_sight(occ, a_cell, b_cell):
    """True if a straight line between two grid cells doesn't cross any
    inflated-occupied cell -- used by _shortcut to collapse a jagged
    cell-by-cell A* path (which hugs the inflated obstacle boundary one
    0.1m step at a time) into fewer, longer, more natural legs."""
    x0, y0 = a_cell
    x1, y1 = b_cell
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    for i in range(steps + 1):
        t = i / steps
        cx = int(round(x0 + (x1 - x0) * t))
        cy = int(round(y0 + (y1 - y0) * t))
        if occ[cy, cx]:
            return False
    return True


def _shortcut(occ, cell_path):
    """Greedy line-of-sight shortcutting: from each waypoint, jump to the
    farthest later cell in the path still reachable by a clear straight
    line, skipping the small in-between steps A* took hugging the inflated
    boundary. Stays grounded in the real grid the whole way -- this only
    removes unnecessary turns from the real plan, it doesn't invent new
    geometry or fall back to any fixed distance/angle."""
    if len(cell_path) <= 2:
        return cell_path
    result = [cell_path[0]]
    i, n = 0, len(cell_path)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not _line_of_sight(occ, cell_path[i], cell_path[j]):
            j -= 1
        result.append(cell_path[j])
        i = j
    return result


def _simplify(grid, cell_path):
    """Merges collinear runs of grid cells into single waypoints -- an
    8-connected A* path changes direction at the resolution of a single
    0.1m cell, which would otherwise mean the robot re-turning every 10cm
    instead of driving one real leg per actual direction change."""
    if len(cell_path) <= 2:
        return [grid.cell_to_world(*c) for c in cell_path]
    waypoints = [cell_path[0]]
    prev_dir = None
    for i in range(1, len(cell_path)):
        cur_dir = (cell_path[i][0] - cell_path[i - 1][0], cell_path[i][1] - cell_path[i - 1][1])
        if prev_dir is not None and cur_dir != prev_dir:
            waypoints.append(cell_path[i - 1])
        prev_dir = cur_dir
    waypoints.append(cell_path[-1])
    return [grid.cell_to_world(*c) for c in waypoints]


def plan_path(grid, start_xy, goal_xy, radius):
    """8-connected A* over the live grid. UNKNOWN cells are treated as
    traversable (optimistic -- the same assumption real occupancy-grid
    navigation stacks make; replanning by the caller on any detected block
    is what corrects for it if a plan turns out to cross something real
    once seen). `radius` is the rotation-swept inflation radius for the
    current footprint (see rotation_swept_radius) -- every waypoint in a
    successfully-returned path is therefore already safe to stop and
    rotate at, by construction, with no separate check needed. Returns a
    simplified list of (x, y) world waypoints, or None if no path exists."""
    occ = _inflated(grid.cells, radius)
    start = grid.world_to_cell(*start_xy)
    goal = grid.world_to_cell(*goal_xy)
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None
    # Never let the start or goal cell itself be blocked by inflation --
    # the robot is already there / genuinely needs to reach it.
    occ[start[1], start[0]] = False
    occ[goal[1], goal[0]] = False

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_set = [(heuristic(start, goal), start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        for nx, ny, step_cost in _neighbors(*current):
            if not grid.in_bounds(nx, ny) or occ[ny, nx]:
                continue
            neighbor = (nx, ny)
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative + heuristic(neighbor, goal), neighbor))

    if goal != start and goal not in came_from:
        return None
    path = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    path = _shortcut(occ, path)
    return _simplify(grid, path)


def plan_to_clear(grid, start_xy, radius):
    """Minimal-displacement recovery: Dijkstra outward from start_xy,
    stopping at the FIRST cell (in order of real accumulated travel cost,
    not straight-line distance) where the given radius no longer overlaps
    anything occupied. Unlike plan_path, this accepts ANY sufficiently-
    clear cell as a valid goal rather than one fixed target, so the
    returned path is provably the true minimum-displacement route to
    safety -- not a heuristic guess at a direction (confirmed live that
    'move away from the nearest occupied point' can walk straight into a
    neighboring obstacle when the real obstacle is an extended structure,
    e.g. a shelf's row of corner legs, not a single point).

    Traversal here deliberately ignores occupancy when choosing which
    cells to expand through (unlike plan_path) -- the start position is,
    by definition, inside blocked space when this is called, so the
    search has to be allowed to move through occupied cells to reach the
    boundary; it only checks whether each cell it settles on is itself
    clear, which is what actually matters for stopping there safely.

    Returns a simplified list of (x, y) world waypoints to the nearest
    clear cell (or [start_xy] if already clear), or None if no clear cell
    is reachable within the grid at all -- a genuinely stuck condition,
    not something a bigger fixed backup distance would ever fix."""
    occ = _inflated(grid.cells, radius)
    start = grid.world_to_cell(*start_xy)
    if not grid.in_bounds(*start):
        return None
    if not occ[start[1], start[0]]:
        return [start_xy]

    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()
    goal = None
    while open_set:
        cost, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        cx, cy = current
        if not occ[cy, cx]:
            goal = current
            break
        for nx, ny, step_cost in _neighbors(cx, cy):
            if not grid.in_bounds(nx, ny):
                continue
            neighbor = (nx, ny)
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative, neighbor))

    if goal is None:
        return None
    path = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    path = _shortcut(occ, path)
    return _simplify(grid, path)
