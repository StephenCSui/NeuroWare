"""Two-robot multi-object delivery demo -- Robot and Robot2 move all 4
payload objects from shelf 1 to shelf 2. 3 of the 4 objects are light
enough for one robot to lift alone (Cube_02 2kg, Cylinder 7.07kg, Cube
9.8kg); Cube_01 (13.44kg) needs both robots straddling it together, same
as before. Robot (spawns near +X, entering in -X) solo-carries the two
objects nearer its own side (Cube, then Cylinder); Robot2 (spawns near
-X, entering +X) solo-carries the one nearer its side (Cube_02). Both
then converge on Cube_01 for the cooperative lift. Each robot works
through its own object list sequentially (not literally simultaneously)
before the cooperative phase starts.

Navigation is generalized turn-to-exact-bearing-then-straight-line
(run_goto) instead of the single hardcoded shelf1->shelf2 route from the
original two-object-only version -- this is what lets a robot return to
shelf 1 for its next object and lets the two robots converge on Cube_01
from wherever their solo work left them, not just from initial spawn.
Still no obstacle avoidance -- straight-line only, matching the earlier
decision that a live bearing-correction approach is less reliable than
an exact-known-heading turn followed by an uncorrected straight drive.

The plate's world-frame orientation (long axis parallel to the shelves'
long axis -- required for it to slide under/out without jamming) is held
fixed continuously for the entire autonomous run via apply_yaw_compensation
(see sim_tick), not just during the Cube_01 carry -- so it's always
correctly oriented for the next pickup or delivery regardless of which
way the chassis is currently facing.

Triggered by pressing P (matches pickup_demo.py's convention -- autonomous
sequences run on keypress, not automatically on launch).

Uses the same low-VRAM MinimalRendering setup as robot_smoke_test.py."""
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
    "width": 960,
    "height": 540,
    "window_width": 1000,
    "window_height": 640,
})

import math
import os
import re
import shutil
import time

import numpy as np
import carb
import carb.settings
import omni.timeline
import omni.appwindow
import carb.input
from omni.physx import get_physx_scene_query_interface
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.sensors.camera import Camera
from pxr import PhysxSchema, UsdGeom, UsdPhysics, Gf

import occupancy_grid

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "two_robot_two_shelf.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[two robot pickup] opened {USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World"])

for path in ["/World/Robot/Geometry/base_link", "/World/Robot2/Geometry/base_link"]:
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
    art_api.CreateSolverPositionIterationCountAttr(255)
    art_api.CreateSolverVelocityIterationCountAttr(64)

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

def setup_robot(articulation_path):
    r = Articulation(articulation_path)
    r.set_link_masses([40.0], link_indices=r.get_link_indices(["base_link"]))
    wj = r.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
    r.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wj)
    r.set_dof_velocity_targets(0, dof_indices=wj)
    # roll/pitch and yaw all get real position control now -- roll/pitch
    # hold level (their starting position) so the plate doesn't tilt under
    # an off-center/heavy object's weight, yaw actively counters the
    # chassis's own turning so the plate isn't dragged around with it.
    # All three were originally damping-only, which can't resist a
    # sustained torque, only slow down a moving one.
    rp = r.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=rp)
    rp_base = r.get_dof_positions(dof_indices=rp).numpy()[0].tolist()
    r.set_dof_position_targets(rp_base, dof_indices=rp)
    yj = r.get_dof_indices(["ball_yaw_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=yj)
    yaw_joint_pos = float(r.get_dof_positions(dof_indices=yj).numpy()[0, 0])
    r.set_dof_position_targets(yaw_joint_pos, dof_indices=yj)
    pj = r.get_dof_indices(["centered_piston_prismatic_z"])
    r.set_dof_limits(lower=0.0, upper=2.0, dof_indices=pj)
    r.set_dof_gains(stiffnesses=20000.0, dampings=1000.0, dof_indices=pj)
    piston_pos = float(r.get_dof_positions(dof_indices=pj).numpy()[0, 0])
    r.set_dof_position_targets(piston_pos, dof_indices=pj)
    return r, wj, rp, yj, yaw_joint_pos, pj, piston_pos

robot, wheel_joints, tilt_joints, yaw_joint, yaw_joint_base, piston_joint, piston_target = setup_robot("/World/Robot/Geometry/base_link")
robot2, wheel_joints2, tilt_joints2, yaw_joint2, yaw_joint_base2, piston_joint2, piston_target2 = setup_robot("/World/Robot2/Geometry/base_link")

PLATE_PATH = "actuator_outer_cylinder_link/piston_rod_link/ball_joint_center_link/ball_roll_link/ball_pitch_link/ball_yaw_link/contact_plate_link"
plate1 = RigidPrim(f"/World/Robot/Geometry/base_link/{PLATE_PATH}")
plate2 = RigidPrim(f"/World/Robot2/Geometry/base_link/{PLATE_PATH}")

# 4 depth cameras per robot, one at each chassis corner, parented directly
# under base_link so they ride along automatically with the chassis's own
# physics motion. Replaces the earlier single forward-facing camera --
# direct fix for a real, confirmed live failure this session: a corner-leg
# collision during an in-place turn that a single forward camera could
# never see (off to the side, not ahead of it). Positions come from the
# real chassis geometry in base.usda, not guessed: chassis_visual is a
# 0.3x0.15x0.04 box centered at local (0,0,0.064) -> half-extents
# (0.15, 0.075), top around z=0.084 -- same corner coordinates used for
# the original single-camera mount, just at all 4 corners instead of one.
#
# Each camera's yaw is offset +-45/+-135 degrees from the chassis forward
# axis (local +X, confirmed as forward earlier via wheel-link positions)
# so 4 x 90-degree FOVs (see set_focal_length below) tile the full 360
# degrees around the chassis exactly, no gaps, overlapping only at the
# shared boundary in each cardinal direction:
#   front-left  (+0.15,+0.075) yaw +45  -> covers [  0, 90]
#   front-right (+0.15,-0.075) yaw -45  -> covers [-90,  0]
#   rear-left   (-0.15,+0.075) yaw +135 -> covers [ 90,180]
#   rear-right  (-0.15,-0.075) yaw -135 -> covers [180,270] (== [-180,-90])
#
# Level (0deg tilt), not angled down -- a 30deg-down mount was tried first
# but only ever saw the floor a few inches ahead, missing everything at
# actual chassis height (the other robot, shelf structure).
#
# Camera's default camera_axes="world" convention (used for both
# set_world_pose and set_local_pose) is +Z up / +X forward -- confirmed by
# reading camera.py directly, NOT the usual USD -Z-forward convention. So
# a pure yaw rotation about local Z is exactly what's needed to point each
# corner camera at its own outward diagonal.
CAMERA_HEIGHT = 0.10  # unchanged from the original single-camera mount
CAMERA_CORNERS = [
    ("FL", np.array([0.15, 0.075, CAMERA_HEIGHT]), 45.0),
    ("FR", np.array([0.15, -0.075, CAMERA_HEIGHT]), -45.0),
    ("RL", np.array([-0.15, 0.075, CAMERA_HEIGHT]), 135.0),
    ("RR", np.array([-0.15, -0.075, CAMERA_HEIGHT]), -135.0),
]

def _yaw_quat(deg):
    """Pure yaw rotation about local Z in the (w,x,y,z) convention this
    project's cameras use (identity = facing local +X, +Z up)."""
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])

def add_depth_camera(base_link_path, name, translation, orientation):
    cam = Camera(
        prim_path=f"{base_link_path}/{name}",
        translation=translation,
        orientation=orientation,
        frequency=20,
        resolution=(128, 128),
    )
    cam.initialize()
    # USD cameras default to a clippingRange around (1, 1e6) -- anything
    # closer than 1m is invisible. Fine for the earlier bare-cube tests
    # (target was 3-4m away), but this robot is ~0.3m and its nearest
    # geometry (the ground right below/ahead) sits well under 1m, so the
    # default left every single pixel reading `inf` -- confirmed directly
    # (raw distance_to_camera array: 0/16384 finite). Not a render-product
    # or annotator-attachment bug, despite how that looked at first.
    cam.set_clipping_range(near_distance=0.02, far_distance=50.0)
    # Default USD camera aperture/focal-length combo (20.955mm / 50mm) is a
    # normal-lens still-camera FOV -- about 24 degrees horizontal, a narrow
    # soda-straw view. Shortening the focal length widens it to roughly 90
    # degrees, which is what makes the 4-corner layout above tile a full
    # 360 degrees with no gaps -- and what the occupancy grid's
    # ray-marching update (occupancy_grid.update_from_camera) needs to see
    # anything off to the side, not just directly ahead.
    cam.set_focal_length(10.5)
    cam.add_distance_to_camera_to_frame()
    for _ in range(30):
        simulation_app.update()
    return cam

def add_corner_cameras(base_link_path):
    return [
        add_depth_camera(base_link_path, f"DepthCamera_{suffix}", pos, _yaw_quat(yaw))
        for suffix, pos, yaw in CAMERA_CORNERS
    ]

depth_cams1 = add_corner_cameras("/World/Robot/Geometry/base_link")
depth_cams2 = add_corner_cameras("/World/Robot2/Geometry/base_link")

def min_depth_over(cams):
    """Closest finite distance across a LIST of cameras, and which camera
    saw it -- lets obstacle checks cover all 4 corner cameras instead of
    only whatever single direction one forward camera used to cover."""
    best_dist, best_cam = None, None
    for cam in cams:
        d = min_depth(cam)
        if d is not None and (best_dist is None or d < best_dist):
            best_dist, best_cam = d, cam
    return best_dist, best_cam

# Diagnostic depth-frame dumps -- lets a sensing question ("did the camera
# really see X") be answered by directly looking at what the sensor saw,
# instead of only numeric min-distance/raycast summaries (which is all
# identify_obstacle/min_depth give you). Each script run gets its own
# timestamped folder; only the most recent MAX_DEPTH_RUNS runs' folders
# are kept, older ones are deleted automatically on the next launch, so
# this doesn't grow unbounded across repeated live/headless sessions.
# Only ever deletes its own "run_*" subfolders under depth_frames/, never
# anything else.
DEPTH_FRAME_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "depth_frames")
MAX_DEPTH_RUNS = 3
DEPTH_VIS_MAX = 3.0  # meters -- clip range for the false-color image; this
                     # robot's obstacle-relevant range is a few meters, not
                     # the camera's full 50m far clip, so clipping here
                     # keeps the visible contrast meaningful.

def _prepare_depth_frame_run_dir():
    os.makedirs(DEPTH_FRAME_ROOT, exist_ok=True)
    existing = sorted(
        d for d in os.listdir(DEPTH_FRAME_ROOT)
        if d.startswith("run_") and os.path.isdir(os.path.join(DEPTH_FRAME_ROOT, d))
    )
    for old in existing[: max(0, len(existing) - (MAX_DEPTH_RUNS - 1))]:
        shutil.rmtree(os.path.join(DEPTH_FRAME_ROOT, old))
        print(f"[two robot pickup] pruned old depth-frame run: {old}")
    run_dir = os.path.join(DEPTH_FRAME_ROOT, "run_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    print(f"[two robot pickup] depth frames for this run -> {run_dir}")
    return run_dir

DEPTH_FRAME_DIR = _prepare_depth_frame_run_dir()

def dump_depth_frame(cam, tag):
    """Saves the camera's current raw distance_to_camera frame as a
    false-color PNG (closer = brighter) under DEPTH_FRAME_DIR. inf/nan
    (nothing in view along that ray) is mapped to the far end rather than
    left blank, so 'nothing there' reads as visually distinct from 'very
    close'. Returns the saved path, or None if no depth data is available
    yet."""
    d = raw_depth_frame(cam)
    if d is None:
        return None
    d = np.asarray(d, dtype=float)
    d = np.where(np.isfinite(d), d, DEPTH_VIS_MAX)
    d = np.clip(d, 0.0, DEPTH_VIS_MAX)
    # tag often embeds a label like "Robot carry /Cube" (object paths have
    # their own leading slash) -- sanitize to a single flat filename
    # component so it never gets interpreted as a subdirectory.
    safe_tag = re.sub(r"[^A-Za-z0-9_.-]+", "_", tag)
    path = os.path.join(DEPTH_FRAME_DIR, f"{safe_tag}_{_tick_count:06d}.png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(path, DEPTH_VIS_MAX - d, cmap="inferno", vmin=0.0, vmax=DEPTH_VIS_MAX)
    return path

def dump_grid_frame(path, tag):
    """Renders the shared occupancy grid (free/occupied/unknown) plus a
    planned path as a PNG under DEPTH_FRAME_DIR, reusing the exact same
    run-folder/auto-pruning mechanism as dump_depth_frame -- lets us
    actually see what the planner currently believes the world looks
    like, not just infer it from behavior."""
    cells = nav_grid.cells
    img = np.zeros((*cells.shape, 3), dtype=np.uint8)
    img[cells == occupancy_grid.UNKNOWN] = (60, 60, 60)
    img[cells == occupancy_grid.FREE] = (20, 120, 20)
    img[cells == occupancy_grid.OCCUPIED] = (200, 30, 30)
    if path:
        for wx, wy in path:
            cx, cy = nav_grid.world_to_cell(wx, wy)
            if nav_grid.in_bounds(cx, cy):
                img[cy, cx] = (255, 255, 0)
    safe_tag = re.sub(r"[^A-Za-z0-9_.-]+", "_", tag)
    frame_path = os.path.join(DEPTH_FRAME_DIR, f"grid_{safe_tag}_{_tick_count:06d}.png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # flip vertically so +Y is up in the image, matching the world's own
    # orientation instead of raw array row order.
    plt.imsave(frame_path, np.flipud(img))
    return frame_path

def raw_depth_frame(cam):
    """Full (H, W) depth array, or None if nothing's available yet.

    Deliberately does NOT call cam.get_depth() -- that method checks for
    the wrong internal annotator key ("distance_to_image_plane" instead
    of "distance_to_camera", which is what add_distance_to_camera_to_frame
    actually registers), so it always logs a spurious "not attached"
    warning regardless of success, every single call. With 8 cameras (4
    corners x 2 robots) all calling this every tick, that flooded the
    console badly enough to visibly lag the live GUI and drown out real
    diagnostic output. Going straight to get_current_frame(), the actual
    working path, skips the bug entirely instead of just tolerating it."""
    frame = cam.get_current_frame()
    return frame.get("distance_to_camera") if frame else None

def min_depth(cam):
    """Closest finite distance currently in view, or None if nothing
    finite is in frame (all rays escaping to infinity)."""
    d = raw_depth_frame(cam)
    if d is None:
        return None
    finite = d[np.isfinite(d)]
    return float(finite.min()) if finite.size else None

def identify_obstacle(cam):
    """Raycasts from the camera's actual world position/forward direction
    to find exactly which prim is triggering an obstacle stop -- direct
    ground truth via PhysX instead of reasoning about mount geometry on
    paper, which has been wrong more than once already this session.
    Returns (rigid_body_path, distance) or (None, None) if the ray
    somehow doesn't hit anything (shouldn't happen if min_depth just
    reported something finite and close)."""
    pos, quat = cam.get_world_pose(camera_axes="world")
    w, x, y, z = quat
    qv = np.array([x, y, z])
    v = np.array([1.0, 0.0, 0.0])
    uv = np.cross(qv, v)
    uuv = np.cross(qv, uv)
    fwd = v + 2 * (w * uv + uuv)
    fwd = fwd / np.linalg.norm(fwd)
    hit = get_physx_scene_query_interface().raycast_closest(
        carb.Float3(float(pos[0]), float(pos[1]), float(pos[2])),
        carb.Float3(float(fwd[0]), float(fwd[1]), float(fwd[2])),
        10.0,
        bothSides=True,
    )
    print(f"[two robot pickup] identify_obstacle: cam pos={pos} fwd={fwd} raw hit dict={hit}")
    if hit and hit.get("rigidBody"):
        return hit["rigidBody"], hit["distance"]
    return None, None

# Shared live occupancy grid -- ONE instance, updated by both robots'
# cameras (sim_tick, below), since they occupy the same physical space and
# each robot benefits from what the other has already seen. Deliberately
# NOT seeded from known shelf geometry -- see occupancy_grid.py's module
# docstring for why (this is what lets a live-placed/moved obstacle,
# spawn_obstacle() or by hand, actually change the plan).
nav_grid = occupancy_grid.OccupancyGrid()

# Both robots enter from a SHORT side of the shelf (X direction), not the
# front/long side -- the plate is a rectangle (0.105 x 0.08), and entering
# via the long side puts the plate's long axis perpendicular to the
# shelf's long axis, which jams it against the shelf structure. Entering
# via a short side keeps the plate's long axis parallel to the shelf's,
# which is what actually fits. Robot enters from the right (-X), Robot2
# from the left (+X), converging on Cube_01's two ends.
TARGET_X = -0.209   # Robot's target (right end of Cube_01), approaching in -X
TARGET_X2 = -0.809  # Robot2's target (left end of Cube_01), approaching in +X
# Cube_01 was lengthened from 0.6m -> 1.0m -> 0.8m (1.0m clipped the
# shelf's corner leg), specifically to put more physical separation
# between the two robots -- at the original 0.6m length they picked it up
# nose-to-nose with ~0.09m of chassis clearance, and turning in place from
# there rammed them into each other. Depth/height were also trimmed
# (0.3->0.28, 0.1->0.06) to bring mass back down after the length change
# raised it -- currently 13.44kg (fixed cross-section, volume scales with
# length/depth/height x density 1000).
#
# Shelf 2 is shifted +4m in Y from shelf 1, X range unchanged. Each solo
# object is delivered to shelf 2 at the SAME X it had on shelf 1 -- the
# original shelf-1 spacing between all 4 objects was already collision-free,
# so mirroring it onto shelf 2 avoids having to work out new drop slots.
DELIVERY_YAW = math.pi / 2
DELIVERY_Y = 3.25
Base_speed = 5.0
Piston_speed = 0.15
LIFT_TARGET = 0.18  # slightly reduced from 0.2 per live observation
pickup_done = False

# Which object (if any) each robot currently has on its plate -- set right
# after a lift completes, cleared right after a lower completes. Used by
# the periodic TRACE print so it knows what object pose to report
# alongside each robot's, and only while something's actually being
# carried.
carrying = {"Robot": None, "Robot2": None}

# Ground-truth cargo tracking -- NOT sensor-based (no camera/depth
# involved at all), just a direct read of the carried object's own
# simulated world Z against the height it was actually lifted to.
# Confirmed necessary live: the depth camera has no idea what it's
# carrying vs. what's an external obstacle, so when a carried object got
# knocked off the plate during a sharp detour turn, the robot kept
# driving with nothing on the plate, then eventually depth-detected its
# own dropped cargo lying nearby and reported it as a new "obstacle" --
# giving a misleading diagnosis and never once registering that delivery
# had already failed. This check gives the robot (and the log) an honest,
# always-on answer to "am I actually still carrying this" independent of
# whatever the depth sensor thinks it sees.
carried_height = {"Robot": None, "Robot2": None}
CARGO_DROP_TOLERANCE = 0.08  # meters -- the actual observed drop was far
                              # larger (~0.35m, carry height down to floor
                              # height), so this comfortably separates a
                              # real fall from ordinary carry-height jitter.

# check_cargo() compares against the height an object was lifted TO --
# correct while it's being held rigid at that height (navigating between
# shelves), but during the deliberate piston-lower ramp at the end of
# each cycle the object's real height is SUPPOSED to decrease, which
# would otherwise look identical to a real drop. Confirmed live: the
# Cylinder leg triggered a "cargo lost" warning purely from its own
# intentional lowering motion, even though it landed exactly on target
# (z after lowering matched expected to the mm). Suspended around both
# piston-lower ramps (solo and cooperative) so only an actual mid-carry
# drop trips it.
_cargo_check_suspended = False

def check_cargo():
    """Called every tick (sim_tick) regardless of phase -- turning,
    driving, detouring -- so cargo loss is caught the instant it happens,
    not inferred later from a confused depth reading. If a carried
    object's Z has sagged past CARGO_DROP_TOLERANCE from its just-lifted
    height, marks it as no longer carried (clears both carrying/
    carried_height for that robot) and prints a clear warning. No-op
    while _cargo_check_suspended (see above)."""
    if _cargo_check_suspended:
        return
    for label, obj_path in list(carrying.items()):
        if obj_path is None:
            continue
        expected_z = carried_height[label]
        z = float(RigidPrim(obj_path).get_world_poses()[0].numpy()[0, 2])
        if abs(z - expected_z) > CARGO_DROP_TOLERANCE:
            print(f"[two robot pickup] {label} WARNING: cargo lost (ground-truth check) -- {obj_path} z={z:.4f}, expected ~{expected_z:.4f}")
            carrying[label] = None
            carried_height[label] = None

# Real path planning replaced the old reactive fixed-angle/fixed-distance
# detour this session (see occupancy_grid.py and run_goto) -- only applied
# on CARRY legs (driving to an empty destination), never APPROACH legs
# (driving to pick up an object), since on approach the object itself
# would show up as "an obstacle" and the robot would stop short of the
# thing it's supposed to reach.
OBSTACLE_THRESHOLD = 0.4   # meters -- immediate hard safety stop if
                            # something's this close ahead RIGHT NOW,
                            # regardless of what the current plan says.
                            # Not routing logic -- a backstop, forces the
                            # next loop iteration in run_goto to replan.
STALL_ABORT_TICKS = 300    # ~5s of yaw genuinely not changing during a
                            # waypoint turn -- long enough to rule out
                            # normal turning just being slow, short enough
                            # not to burn most of run_turn's 3000-step
                            # budget wedged before giving up on this hop.
                            # A failed turn is treated as a genuine
                            # anomaly (see execute_path) that triggers
                            # real minimal-displacement replanning, not a
                            # fixed-distance reflex.

def get_xy_yaw(r):
    pos, quat = r.get_world_poses()
    p = pos.numpy()[0]
    w, x, y, z = quat.numpy()[0]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return p[0], p[1], yaw

RAMP_STEPS = 90  # ~1.5s to reach full speed from a standstill
DECEL_DISTANCE = 0.3  # meters -- start slowing a drive within this much of
                        # the target -- symmetric counterpart to RAMP_STEPS,
                        # see drive_to_point_step.
MIN_RAMP_OUT = 0.6     # floor on the decel ramp's speed multiplier. Raised
                        # from an original 0.15 (0.75 rad/s wheel target)
                        # after headless testing this session isolated the
                        # real cause of carry-turn stalls: below roughly
                        # 2 rad/s commanded wheel velocity, the wheel drive
                        # (damping-only, no stiffness) genuinely stops
                        # tracking its own target -- confirmed directly via
                        # get_dof_velocities readback showing wrong-signed,
                        # multiples-of-target wheel speeds, reproduced in a
                        # fully isolated empty scene with no cargo and no
                        # obstacles (see _diag_wheel_index.py), and via
                        # repeated real carry-turn runs that stalled at
                        # ~5.4-5.5 deg short of target every time regardless
                        # of cargo weight or nearby geometry -- i.e. this
                        # was never a "ramp too weak to finish the last bit"
                        # problem, it was a genuine low-speed control
                        # instability. 0.6 (3.0 rad/s floor) keeps the turn
                        # comfortably clear of that zone while still giving
                        # a real deceleration ramp (verified: turn completes
                        # in the same ~307 ticks as no ramp at all, with a
                        # tighter final error, 0.04 deg vs 0.11 deg).

DECEL_ANGLE = math.radians(15)  # start slowing a turn within this much of
                                  # the target yaw -- symmetric counterpart
                                  # to the existing RAMP_STEPS accel-in ramp,
                                  # same min(1.0, x/const) pattern, so a turn
                                  # eases to its stop instead of snapping
                                  # from full ramped speed straight to zero.
                                  # A motor-control tuning constant, not a
                                  # routing decision -- unlike the old fixed
                                  # detour turn angle/hop distance it isn't
                                  # standing in for real planning.

def turn_to_yaw_step(r, wj, target_yaw, step=RAMP_STEPS, speed=Base_speed, yaw_tol=0.002):
    """One control tick of pure in-place rotation toward an EXACT absolute
    yaw angle (not a bearing recomputed each tick) -- no forward motion.
    yaw_tol is tight (~0.3 degrees) so the straight drive that follows
    genuinely is straight, not a shallow curve. Returns True once aligned
    within yaw_tol (and stops the robot)."""
    ramp_in = min(1.0, step / RAMP_STEPS)
    _, _, yaw = get_xy_yaw(r)
    yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
    if abs(yaw_err) <= yaw_tol:
        r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
        return True
    ramp_out = max(MIN_RAMP_OUT, min(1.0, abs(yaw_err) / DECEL_ANGLE))
    speed = speed * min(ramp_in, ramp_out)
    turn = 1.0 if yaw_err > 0 else -1.0
    left, right = -turn * speed, turn * speed
    r.set_dof_velocity_targets([left, right, left, right], dof_indices=wj)
    return False

def bearing_to(x, y, target_x, target_y):
    return math.atan2(target_y - y, target_x - x)

def drive_to_point_step(r, wj, target_x, target_y, step=RAMP_STEPS, speed=Base_speed, stop_dist=0.01):
    """One control tick of driving dead straight (equal wheel speeds, no
    steering correction) until forward progress toward (target_x, target_y)
    is used up. Only valid once the robot is already facing the exact
    bearing toward that point (call turn_to_yaw_step to completion first).

    Stop condition is a PROJECTION, not raw distance-to-target: remaining =
    (target - pos) . (cos(yaw), sin(yaw)) -- how much further forward
    (along the robot's actual current heading) is left before reaching the
    target's position along that line. This was raw Euclidean distance
    originally (dist <= stop_dist), which is the same undershoot-prone
    symmetric-check mistake in disguise: raw distance is only monotonically
    decreasing if the straight line passes EXACTLY through the target, and
    a locked-in heading is never exactly the bearing (real settle drift was
    ~0.2 degrees in testing) -- over a several-meter drive that produces a
    perpendicular miss bigger than stop_dist, so dist bottoms out just
    above the threshold and then starts climbing again as the robot drives
    past, blowing straight through the step budget (caught by headless
    smoke test: the Cylinder leg missed by ~0.014m and consequently drove
    ~2m past the target before giving up). The projection is immune to any
    perpendicular miss -- it only tracks progress along the line actually
    being driven, so it hits zero (reached-or-passed) regardless. Returns
    True once arrived (and stops the robot)."""
    ramp_in = min(1.0, step / RAMP_STEPS)
    x, y, yaw = get_xy_yaw(r)
    remaining = (target_x - x) * math.cos(yaw) + (target_y - y) * math.sin(yaw)
    if remaining <= stop_dist:
        r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
        return True
    # Symmetric decel-out, same min(1.0, x/const) pattern as the accel-in
    # ramp and turn_to_yaw_step's DECEL_ANGLE -- eases to a stop instead of
    # driving at full speed until it hits stop_dist and snapping to zero.
    ramp_out = max(MIN_RAMP_OUT, min(1.0, remaining / DECEL_DISTANCE))
    speed = speed * min(ramp_in, ramp_out)
    r.set_dof_velocity_targets([speed] * 4, dof_indices=wj)
    return False

SETTLE_FRAMES = 60  # ~1s buffer after each turn before the next stage starts

def apply_yaw_compensation(r, yj, yaw_base, chassis_yaw0):
    """Actively holds the plate's world-frame yaw constant regardless of
    how much the chassis underneath it has turned since chassis_yaw0 --
    without this, the plate (only damped before) just gets dragged around
    with the chassis, putting torque into whatever's resting on it."""
    _, _, chassis_yaw = get_xy_yaw(r)
    delta = math.atan2(math.sin(chassis_yaw - chassis_yaw0), math.cos(chassis_yaw - chassis_yaw0))
    r.set_dof_position_targets(yaw_base - delta, dof_indices=yj)

# Captured ONCE, before any autonomous movement, and never reset -- yaw
# compensation runs continuously (via sim_tick) for the entire multi-object
# run, not just during the Cube_01 carry, so the plate's world orientation
# (long axis parallel to the shelves' long axis) stays correct at every
# pickup and delivery, not just the ones that used to be hardcoded.
_, _, chassis_yaw0 = get_xy_yaw(robot)
_, _, chassis_yaw0_2 = get_xy_yaw(robot2)

_tick_count = 0
DEPTH_PRINT_INTERVAL = 60   # ~1s at 60fps -- proving the sensor stays live
                            # throughout the run without flooding the console
TRACE_INTERVAL = 600        # ~10s at 60fps, per your ask

KNOWN_OBJECTS = ["/Cube", "/Cube_01", "/Cube_02", "/Cylinder"]
obstacle_paths = []  # appended by spawn_obstacle

# Live state export for the separate 2D visualizer (isaac/visualize_2d.py) --
# a genuinely independent process, not sharing memory with this one, so it
# reads a small snapshot file instead. Written to a .tmp path and atomically
# renamed into place (os.replace) so the visualizer never reads a half-written
# file mid-save.
VIS_STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_vis_state.npz")
CURRENT_PLAN = {"Robot": None, "Robot2": None}  # set by run_goto whenever a
                                                  # fresh path is computed, so
                                                  # the visualizer can draw
                                                  # exactly what the planner
                                                  # currently believes.

def export_vis_state():
    rx, ry, ryaw = get_xy_yaw(robot)
    r2x, r2y, r2yaw = get_xy_yaw(robot2)
    obj_names, obj_xy = [], []
    for path in KNOWN_OBJECTS + obstacle_paths:
        pos = RigidPrim(path).get_world_poses()[0].numpy()[0]
        obj_names.append(path)
        obj_xy.append((float(pos[0]), float(pos[1])))
    plan_r = np.array(CURRENT_PLAN["Robot"], dtype=np.float64) if CURRENT_PLAN["Robot"] else np.zeros((0, 2))
    plan_r2 = np.array(CURRENT_PLAN["Robot2"], dtype=np.float64) if CURRENT_PLAN["Robot2"] else np.zeros((0, 2))
    tmp_path = VIS_STATE_PATH + ".tmp"
    with open(tmp_path, "wb") as f:
        np.savez(
            f,
            grid=nav_grid.cells,
            robot=np.array([rx, ry, ryaw]),
            robot2=np.array([r2x, r2y, r2yaw]),
            plan_robot=plan_r,
            plan_robot2=plan_r2,
            obj_names=np.array(obj_names),
            obj_xy=np.array(obj_xy) if obj_xy else np.zeros((0, 2)),
            carrying_robot=carrying["Robot"] or "",
            carrying_robot2=carrying["Robot2"] or "",
        )
    os.replace(tmp_path, VIS_STATE_PATH)

def print_trace():
    """Robot pose for both robots, plus EVERY known box (all 4 payload
    objects + any spawned obstacles), not just whatever's currently on a
    plate -- ground truth for "did it actually avoid the obstacle", not
    just "did the script eventually print all 4 objects delivered" (which
    turned out to be printed even when a robot was stuck wedged against
    an obstacle for the entire rest of the run -- the script has no hard
    failure check, it just exhausts step budgets and moves on)."""
    for label, r in [("Robot", robot), ("Robot2", robot2)]:
        x, y, yaw = get_xy_yaw(r)
        print(f"[two robot pickup] TRACE {label}: pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f}")
    for path in KNOWN_OBJECTS + obstacle_paths:
        prim = RigidPrim(path)
        pos = prim.get_world_poses()[0].numpy()[0]
        _, _, oyaw = get_xy_yaw(prim)
        held_by = next((lbl for lbl, p in carrying.items() if p == path), None)
        tag = f" (on {held_by}'s plate)" if held_by else ""
        print(f"[two robot pickup] TRACE {path}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) yaw={math.degrees(oyaw):.1f}{tag}")

def sim_tick():
    global _tick_count
    apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
    apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
    simulation_app.update()
    check_cargo()
    for cam in depth_cams1:
        occupancy_grid.update_from_camera(nav_grid, cam, raw_depth_frame(cam))
    for cam in depth_cams2:
        occupancy_grid.update_from_camera(nav_grid, cam, raw_depth_frame(cam))
    _tick_count += 1
    if _tick_count % 5 == 0:
        export_vis_state()
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        d1, _ = min_depth_over(depth_cams1)
        d2, _ = min_depth_over(depth_cams2)
        d1s = f"{d1:.3f}" if d1 is not None else "nothing in view"
        d2s = f"{d2:.3f}" if d2 is not None else "nothing in view"
        print(f"[two robot pickup] depth -- Robot closest: {d1s}  Robot2 closest: {d2s}")
    if _tick_count % TRACE_INTERVAL == 0:
        print_trace()
        for i, cam in enumerate(depth_cams1):
            dump_depth_frame(cam, f"Robot_periodic_{i}")
        for i, cam in enumerate(depth_cams2):
            dump_depth_frame(cam, f"Robot2_periodic_{i}")

def run_turn(r, wj, target_yaw, label="", stall_abort_ticks=None, cam=None):
    # 3000-step budget (not the original 1500) -- the original two-object
    # version only ever turned ~90 degrees (fixed pickup headings -> +Y).
    # Generalized navigation between arbitrary objects can require turns
    # close to 180 degrees (e.g. Cube's delivery heading back around to
    # Cylinder's approach bearing is ~184 degrees), which didn't reliably
    # finish within 1500 steps -- caught by headless smoke test, where the
    # turn silently ran out of budget mid-rotation with no warning (unlike
    # run_drive) and the following drive then failed too.
    #
    # stall_abort_ticks: if given, bail out early (returning False) once
    # yaw has genuinely not changed for this many consecutive ticks,
    # instead of continuing to burn through the full step budget wedged
    # against something. execute_path treats a False return as a genuine
    # anomaly and triggers real minimal-displacement replanning -- no
    # amount of continuing to try to turn in place frees a chassis once
    # it's physically wedged, it has to move away first. Normal
    # (approach-leg) turns leave this None so a merely-slow, not stuck,
    # turn is never aborted early.
    #
    # Returns True if target_yaw was reached, False if aborted on stall or
    # the step budget ran out.
    #
    # Yaw-compensation (apply_yaw_compensation, driven every tick by
    # sim_tick) is disabled for the duration of the turn -- confirmed via
    # isolated headless testing (see _diag_wheel_index.py) that actively
    # driving the ball-yaw joint (position OR pure-damping drive, at any
    # gain from 5000 down to 0 stiffness) while the chassis turns causes a
    # real, deterministic wheel-velocity blowup (readbacks 9-12x the
    # commanded target, wrong sign on two wheels) in one rotation
    # direction. The plate is a free (undriven) joint during the turn, so
    # it holds its own world orientation passively via its own inertia
    # (no active torque to fight the chassis with) instead of fighting it
    # -- apply_yaw_compensation resumes immediately once the turn ends,
    # recomputing its target from the ORIGINAL chassis_yaw0 baseline (see
    # apply_yaw_compensation's docstring), so no state needs to be
    # reseeded here, just the gains restored.
    this_yj = yaw_joint if r is robot else yaw_joint2 if r is robot2 else None
    if this_yj is not None:
        r.set_dof_gains(stiffnesses=0.0, dampings=0.0, dof_indices=this_yj)
    start_tick = _tick_count
    start_yaw = get_xy_yaw(r)[2]
    print(f"[two robot pickup] {label} TASK START rotate tick={start_tick} from={math.degrees(start_yaw):.1f} to={math.degrees(target_yaw):.1f} deg")
    prev_yaw = start_yaw
    stall_count = 0
    finished = False
    abort_reason = None
    max_wheel_track_err = 0.0
    for step in range(3000):
        _, _, yaw_now = get_xy_yaw(r)
        d_yaw = math.atan2(math.sin(yaw_now - prev_yaw), math.cos(yaw_now - prev_yaw))
        stall_count = stall_count + 1 if (abs(d_yaw) < 1e-5 and step > RAMP_STEPS) else 0
        if step % 30 == 0:
            # Sampled (not every tick, to keep this cheap) -- how far actual
            # wheel velocity is from what was just commanded, regardless of
            # whether a stall is currently suspected. This is what
            # distinguishes "wheel control isn't tracking its own command"
            # (a real control/physics fault) from "wheels are tracking fine
            # but something external is stopping the chassis" (e.g. blocked).
            actual_vel = r.get_dof_velocities(dof_indices=wj).numpy()[0]
            target_vel = r.get_dof_velocity_targets(dof_indices=wj).numpy()[0]
            track_err = float(np.max(np.abs(actual_vel - target_vel)))
            max_wheel_track_err = max(max_wheel_track_err, track_err)
        if stall_count and stall_count % 200 == 0:
            actual_vel = r.get_dof_velocities(dof_indices=wj).numpy()[0]
            target_vel = r.get_dof_velocity_targets(dof_indices=wj).numpy()[0]
            print(f"[two robot pickup] {label} STALL WARNING at step={step}: yaw={math.degrees(yaw_now):.2f} not changing (stuck {stall_count} ticks) -- wheels actual={actual_vel} target={target_vel}")
            px, py, pz = r.get_world_poses()[0].numpy()[0]
            print(f"[two robot pickup] {label} chassis position check -- pos=({px:.3f},{py:.3f},{pz:.3f}) (settled height should be close to the value at spawn, not sinking/floating)")
            if cam is not None:
                print(f"[two robot pickup] {label} stall carrying-state -- {carrying}")
                for i, one_cam in enumerate(cam):
                    hit_body, hit_dist = identify_obstacle(one_cam)
                    d = min_depth(one_cam)
                    frame_path = dump_depth_frame(one_cam, f"{label.replace(' ', '_')}_stallcam{i}")
                    ds = f"{d:.3f}" if d is not None else "nothing in view"
                    print(f"[two robot pickup] {label} stall cam#{i} -- raycast touching: {hit_body} (dist={hit_dist}), min_depth={ds} [frame: {frame_path}]")
            # Diagnostic: is the yaw-compensation joint (which fights the
            # chassis's own rotation every tick, see apply_yaw_compensation)
            # near a physical limit? That would resist chassis rotation
            # independent of any nearby obstacle -- worth ruling in/out with
            # real data rather than assuming every stall is a collision.
            this_yj, this_yaw_base, this_yaw0 = (
                (yaw_joint, yaw_joint_base, chassis_yaw0) if r is robot
                else (yaw_joint2, yaw_joint_base2, chassis_yaw0_2) if r is robot2
                else (None, None, None)
            )
            if this_yj is not None:
                yj_pos = float(r.get_dof_positions(dof_indices=this_yj).numpy()[0, 0])
                yj_lower, yj_upper = r.get_dof_limits(dof_indices=this_yj)
                yj_lower, yj_upper = float(yj_lower.numpy()[0, 0]), float(yj_upper.numpy()[0, 0])
                delta = math.atan2(math.sin(yaw_now - this_yaw0), math.cos(yaw_now - this_yaw0))
                yj_target = this_yaw_base - delta
                print(f"[two robot pickup] {label} yaw-joint check -- actual_pos={yj_pos:.3f} commanded_target={yj_target:.3f} limits=[{yj_lower:.3f}, {yj_upper:.3f}] (rad)")
        if stall_abort_ticks is not None and stall_count >= stall_abort_ticks:
            print(f"[two robot pickup] {label} ABORTING turn -- stalled for {stall_count} ticks, backing off instead of continuing to burn the step budget")
            r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
            abort_reason = "stalled"
            break
        prev_yaw = yaw_now
        if turn_to_yaw_step(r, wj, target_yaw, step=step):
            finished = True
            break
        sim_tick()
    else:
        print(f"[two robot pickup] WARNING: {label} did not finish turning within the step budget")
        abort_reason = "timeout"
    if this_yj is not None:
        r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=this_yj)
    final_yaw = get_xy_yaw(r)[2]
    final_err_deg = math.degrees(abs(math.atan2(math.sin(target_yaw - final_yaw), math.cos(target_yaw - final_yaw))))
    duration_ticks = _tick_count - start_tick
    if finished:
        result_str = "OK"
        reason_str = "reached target within tolerance"
    else:
        result_str = "FAIL"
        reason_str = (
            f"stalled -- wheels not tracking commanded velocity (max tracking error {max_wheel_track_err:.3f} rad/s), "
            f"chassis yaw stuck {final_err_deg:.2f} deg short of target"
            if abort_reason == "stalled" else
            f"timeout -- step budget exhausted, still {final_err_deg:.2f} deg short of target (max wheel tracking error {max_wheel_track_err:.3f} rad/s over the run)"
        )
    print(f"[two robot pickup] {label} TASK END rotate result={result_str} duration_ticks={duration_ticks} final_yaw_err_deg={final_err_deg:.3f} tolerance_deg={math.degrees(0.002):.3f} max_wheel_track_err={max_wheel_track_err:.3f} reason: {reason_str}")
    for _ in range(SETTLE_FRAMES):
        sim_tick()
    return finished

def run_drive(r, wj, target_x, target_y, label="", cam=None, cargo_key=None):
    """Returns "arrived", "blocked" (only possible if cam is given),
    "cargo_lost" (only possible if cargo_key is given), or "timeout"."""
    print(f"[two robot pickup] {label} driving to ({target_x:.3f},{target_y:.3f})...")
    for step in range(2000):
        if step % 300 == 0:
            x, y, yaw = get_xy_yaw(r)
            dist = math.hypot(target_x - x, target_y - y)
            print(f"[two robot pickup] {label} step={step} x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} dist={dist:.3f}")
        if cargo_key is not None and carrying.get(cargo_key) is None:
            r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
            print(f"[two robot pickup] {label} stopping -- cargo already lost (ground-truth check, see WARNING above), nothing left to deliver on this leg")
            return "cargo_lost"
        if cam is not None:
            # Dynamic threshold, not the old flat constant -- matches
            # whatever the planner itself is currently treating as safe
            # clearance for this footprint (chassis alone, or grown for
            # whatever's being carried). A flat threshold larger than the
            # real footprint radius would trip on routes the planner
            # already knows are genuinely clear, for no reason.
            threshold = current_rotation_radius(cargo_key) if cargo_key is not None else OBSTACLE_THRESHOLD
            d, hit_cam = min_depth_over(cam)
            if d is not None and d < threshold:
                r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
                hit_body, hit_dist = identify_obstacle(hit_cam)
                # Write this real, immediate detection straight into the
                # shared grid -- see occupancy_grid.mark_detected_obstacle's
                # docstring for why the passive per-tick scan alone can
                # lose it, which otherwise leaves the planner replanning
                # the exact same doomed route it was just vetoed on.
                cam_pos, cam_quat = hit_cam.get_world_pose(camera_axes="world")
                occupancy_grid.mark_detected_obstacle(nav_grid, cam_pos, cam_quat, d)
                frame_path = dump_depth_frame(hit_cam, label.replace(" ", "_") + "_blocked")
                print(f"[two robot pickup] {label} obstacle at {d:.3f}m ahead -- stopping (raycast hit: {hit_body}, dist={hit_dist}) [depth frame: {frame_path}]")
                return "blocked"
        if drive_to_point_step(r, wj, target_x, target_y, step=step):
            for _ in range(30):
                sim_tick()
            return "arrived"
        sim_tick()
    print(f"[two robot pickup] WARNING: {label} did not arrive within the step budget")
    for _ in range(30):
        sim_tick()
    return "timeout"

# Real half-extents (meters), queried directly from the scene's actual
# bounding boxes this session (not guessed) -- used to grow the planning
# footprint to cover whatever's currently being carried. See
# occupancy_grid.rotation_swept_radius.
OBJECT_HALF_EXTENTS = {
    "/Cube": (0.1001, 0.1750),
    "/Cube_01": (0.4000, 0.1400),
    "/Cube_02": (0.0500, 0.2000),
    "/Cylinder": (0.1500, 0.1500),
}

def current_rotation_radius(label):
    """Planning inflation radius for `label`'s robot right now -- the
    chassis's own rotation-swept circle, grown by the real half-extents of
    whatever it's currently carrying (per the ground-truth `carrying`
    dict), or just the chassis alone if carrying nothing. Not a single
    fixed constant: what's actually safe to rotate through genuinely
    differs between carrying nothing and carrying Cube_01."""
    obj_path = carrying.get(label)
    if obj_path is None:
        return occupancy_grid.rotation_swept_radius()
    half_x, half_y = OBJECT_HALF_EXTENTS.get(obj_path, (0.0, 0.0))
    return occupancy_grid.rotation_swept_radius(half_x, half_y)

MAX_PLAN_ATTEMPTS = 30  # last-resort safety cap on total outer replans, not
                         # the primary way this gives up -- see run_goto.
                         # The real failure condition is plan_to_clear
                         # itself reporting no reachable clear cell.

def execute_path(r, wj, path, label, cam, cargo_key):
    """Walks the FULL path (every waypoint in sequence), using the same
    turn-then-drive primitives as always. Only stops early on a genuine
    detected anomaly -- the plan was computed against the live grid's
    current state, so as long as nothing about the world changes
    mid-execution, every waypoint should just work, by construction (see
    occupancy_grid.plan_path's inflation). Anomalies:
      "blocked"     -- a turn failed to complete, or the depth sensor
                        tripped mid-drive (something's there that the
                        plan didn't account for).
      "no_progress" -- a hop nominally completed but the chassis ended up
                        within 0.02m of where it started -- execution
                        didn't match what the plan predicted, which means
                        the world-model the plan was built on is wrong,
                        not something to paper over with a fixed reflex.
      "cargo_lost"  -- ground-truth cargo check already cleared this
                        leg's object; nothing left to deliver.
    Returns "arrived" if every waypoint was reached cleanly."""
    for wp_x, wp_y in path[1:]:
        x0, y0, _ = get_xy_yaw(r)
        bearing = bearing_to(x0, y0, wp_x, wp_y)
        turned = run_turn(r, wj, bearing, label=label, stall_abort_ticks=STALL_ABORT_TICKS, cam=cam)
        if not turned:
            return "blocked"
        result = run_drive(r, wj, wp_x, wp_y, label=label, cam=cam, cargo_key=cargo_key)
        if result == "cargo_lost":
            return "cargo_lost"
        if result != "arrived":  # "blocked" or "timeout"
            return "blocked"
        x1, y1, _ = get_xy_yaw(r)
        if math.hypot(x1 - x0, y1 - y0) < 0.02:
            return "no_progress"
    return "arrived"

def run_goto(r, wj, target_x, target_y, label="", cam=None, cargo_key=None):
    """Generalized point-to-point navigation.

    Without cam (APPROACH legs -- driving to an object to pick it up):
    unchanged direct behavior -- turn to the exact bearing once, drive
    straight there. The object being approached would itself register as
    an obstacle to the live grid, so planning is skipped here on purpose,
    same reasoning the old detour scoping used.

    With cam (CARRY legs -- driving to an empty destination): plan ONE
    full route per outer attempt against the shared live occupancy grid
    (nav_grid, built from both robots' depth cameras), using the current
    footprint's real rotation-swept radius (current_rotation_radius) so
    every waypoint the plan returns is already safe to stop and rotate at
    -- no separate clearance check needed anywhere. execute_path then
    walks that ENTIRE route; a fresh replan only happens if execute_path
    reports a genuine anomaly (something actually blocked, or execution
    didn't match the plan), not on every single hop regardless of whether
    anything changed.

    Recovery, when an anomaly happens, is the same planner searching for
    a different goal: occupancy_grid.plan_to_clear finds the true
    minimum-displacement route to the nearest cell that's actually clear
    for the current footprint (a real Dijkstra search, not a heuristic
    direction), which is executed the same way before the outer loop
    retries toward the real target. Giving up only happens when
    plan_to_clear itself reports no reachable clear cell exists --
    MAX_PLAN_ATTEMPTS is just a last-resort backstop against a genuine
    infinite loop, not the primary failure signal.

    cargo_key (if given, only meaningful alongside cam on a carry leg):
    the "Robot"/"Robot2" key into the module-level `carrying` dict. If
    check_cargo() (ground-truth, not sensor-based -- see its own
    docstring) has already cleared that entry, the leg stops immediately
    instead of continuing to plan a route for cargo that's already gone.

    Returns True if the target was actually reached, False otherwise --
    callers that only do something at the real destination (e.g. lowering
    the plate, which is only safe once actually clear of shelf structure)
    need to know the difference between "arrived" and "gave up nearby"."""
    if cam is None:
        x, y, _ = get_xy_yaw(r)
        bearing = bearing_to(x, y, target_x, target_y)
        run_turn(r, wj, bearing, label=label)
        run_drive(r, wj, target_x, target_y, label=label, cargo_key=cargo_key)
        return True

    for attempt in range(MAX_PLAN_ATTEMPTS):
        x, y, _ = get_xy_yaw(r)
        if math.hypot(target_x - x, target_y - y) < 0.05:
            return True
        radius = current_rotation_radius(label)

        path = occupancy_grid.plan_path(nav_grid, (x, y), (target_x, target_y), radius)
        if cargo_key in CURRENT_PLAN:
            CURRENT_PLAN[cargo_key] = path
        grid_frame_path = dump_grid_frame(path, label.replace(" ", "_") + f"_attempt{attempt}")
        if path is None or len(path) < 2:
            print(f"[two robot pickup] {label} WARNING: no path to target found -- attempting minimal-displacement recovery instead [grid frame: {grid_frame_path}]")
            result = "blocked"
        else:
            print(f"[two robot pickup] {label} planned {len(path)}-waypoint path [grid frame: {grid_frame_path}]")
            result = execute_path(r, wj, path, label, cam, cargo_key)
            if result == "arrived":
                return True
            if result == "cargo_lost":
                return False

        print(f"[two robot pickup] {label} {result} -- planning minimal-displacement recovery to nearest clear cell...")
        x, y, _ = get_xy_yaw(r)
        clear_path = occupancy_grid.plan_to_clear(nav_grid, (x, y), radius)
        if cargo_key in CURRENT_PLAN:
            CURRENT_PLAN[cargo_key] = clear_path
        if clear_path is None:
            print(f"[two robot pickup] {label} WARNING: no reachable clear cell found -- genuinely stuck, giving up")
            return False
        if len(clear_path) < 2:
            # plan_to_clear returned just [start_xy] -- the grid itself
            # already considers the current cell clear (not occupied+
            # inflated). The "blocked" trigger came from the real-time
            # reactive sensor threshold, not the grid, so this is a
            # timing mismatch, not an actual dead end -- nothing to move,
            # just loop back and retry the main plan instead of treating
            # it as the same failure as a real exhausted search.
            print(f"[two robot pickup] {label} already clear per the grid -- retrying main plan")
        else:
            recovery_result = execute_path(r, wj, clear_path, f"{label} recovery", cam, cargo_key)
            if recovery_result == "cargo_lost":
                return False
        # loop back to top regardless of recovery outcome -- fresh full
        # replan toward the real target from wherever it ended up.
    print(f"[two robot pickup] {label} WARNING: still not at target after {MAX_PLAN_ATTEMPTS} replans, giving up")
    return False

def solo_pickup_carry_lower(r, wj, pj, piston_start, obj_path, label="", cam=None):
    """Full single-robot cycle: navigate to the object's own measured
    pickup point on shelf 1, lift it alone, carry it to shelf 2 at the
    SAME X, lower it back down, and return the piston to piston_start.
    No re-alignment needed before sliding under the next object -- the
    plate's world orientation is held fixed by continuous yaw compensation
    (sim_tick) for the whole run. Returns the final piston position
    (== piston_start) so the caller can update its own global.

    cam (if given) only guards the CARRY leg, not the approach -- see
    run_goto's docstring for why the approach can't use it."""
    obj = RigidPrim(obj_path)
    obj_x, obj_y = (float(v) for v in obj.get_world_poses()[0].numpy()[0][:2])

    run_goto(r, wj, obj_x, obj_y, label=f"{label} approach {obj_path}")

    print(f"[two robot pickup] {label} lifting {obj_path} alone...")
    z0 = obj.get_world_poses()[0].numpy()[0, 2]
    pt = piston_start
    while pt < LIFT_TARGET:
        pt = min(LIFT_TARGET, pt + Piston_speed * (1 / 60))
        r.set_dof_position_targets(pt, dof_indices=pj)
        sim_tick()
    z1 = obj.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] {label} {obj_path} z: {z0:.4f} -> {z1:.4f} (lifted {z1-z0:.4f})")

    carrying[label] = obj_path
    carried_height[label] = z1
    arrived = run_goto(r, wj, obj_x, DELIVERY_Y, label=f"{label} carry {obj_path}", cam=cam, cargo_key=label)

    z2 = obj.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] {label} {obj_path} z after carry: {z2:.4f} (still on the plate if close to {z1:.4f})")

    if not arrived:
        # Only lower at the actual drop-off point -- the plate is only
        # floating clear of shelf structure at LIFT_TARGET height, and
        # lowering it while still short of the real destination means
        # lowering it straight into whatever it's still floating over
        # (confirmed live: this is what was physically jamming the robot
        # against shelf1's rail structure). Leaves the object held up and
        # `carrying[label]` still set -- it's still genuinely being
        # carried, just not yet delivered.
        print(f"[two robot pickup] {label} WARNING: never reached the drop-off point -- holding {obj_path} up, not lowering here")
        return pt

    print(f"[two robot pickup] {label} lowering {obj_path}...")
    global _cargo_check_suspended
    _cargo_check_suspended = True
    while pt > piston_start:
        pt = max(piston_start, pt - Piston_speed * (1 / 60))
        r.set_dof_position_targets(pt, dof_indices=pj)
        sim_tick()
    for _ in range(30):
        sim_tick()
    _cargo_check_suspended = False
    carrying[label] = None
    carried_height[label] = None
    z3 = obj.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] {label} {obj_path} z after lowering: {z3:.4f} (settled if close to {z0:.4f})")
    return pt

def run_full_delivery():
    global piston_target, piston_target2, pickup_done

    print("[two robot pickup] Robot: solo-carrying Cube, then Cylinder...")
    piston_target = solo_pickup_carry_lower(robot, wheel_joints, piston_joint, piston_target, "/Cube", label="Robot", cam=depth_cams1)
    piston_target = solo_pickup_carry_lower(robot, wheel_joints, piston_joint, piston_target, "/Cylinder", label="Robot", cam=depth_cams1)

    print("[two robot pickup] Robot2: solo-carrying Cube_02...")
    piston_target2 = solo_pickup_carry_lower(robot2, wheel_joints2, piston_joint2, piston_target2, "/Cube_02", label="Robot2", cam=depth_cams2)

    print("[two robot pickup] both robots converging on Cube_01 for the cooperative lift...")
    obj01 = RigidPrim("/Cube_01")
    obj01_y = float(obj01.get_world_poses()[0].numpy()[0, 1])
    run_goto(robot, wheel_joints, TARGET_X, obj01_y, label="Robot")
    run_goto(robot2, wheel_joints2, TARGET_X2, obj01_y, label="Robot2")

    print("[two robot pickup] lifting Cube_01 together...")
    z0 = obj01.get_world_poses()[0].numpy()[0, 2]
    piston_rest, piston_rest2 = piston_target, piston_target2
    while piston_target < LIFT_TARGET:
        piston_target = min(LIFT_TARGET, piston_target + Piston_speed * (1 / 60))
        piston_target2 = min(LIFT_TARGET, piston_target2 + Piston_speed * (1 / 60))
        robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)
        robot2.set_dof_position_targets(piston_target2, dof_indices=piston_joint2)
        sim_tick()
    z1 = obj01.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] Cube_01 z: {z0:.4f} -> {z1:.4f} (lifted {z1-z0:.4f})")
    carrying["Robot"] = carrying["Robot2"] = "/Cube_01"
    carried_height["Robot"] = carried_height["Robot2"] = z1

    print("[two robot pickup] carrying to shelf 2 -- Robot turns first (Robot2 holds)...")
    robot2.set_dof_velocity_targets([0] * 4, dof_indices=wheel_joints2)
    for step in range(1500):
        if turn_to_yaw_step(robot, wheel_joints, DELIVERY_YAW, step=step):
            break
        sim_tick()
    for _ in range(SETTLE_FRAMES):
        sim_tick()

    print("[two robot pickup] Robot2 turns (Robot holds)...")
    robot.set_dof_velocity_targets([0] * 4, dof_indices=wheel_joints)
    for step in range(1500):
        if turn_to_yaw_step(robot2, wheel_joints2, DELIVERY_YAW, step=step):
            break
        sim_tick()
    for _ in range(SETTLE_FRAMES):
        sim_tick()

    print("[two robot pickup] both aligned -- driving straight to shelf 2...")
    r1_done = r2_done = False
    for step in range(2000):
        if not r1_done:
            r1_done = drive_to_point_step(robot, wheel_joints, TARGET_X, DELIVERY_Y, step=step)
        if not r2_done:
            r2_done = drive_to_point_step(robot2, wheel_joints2, TARGET_X2, DELIVERY_Y, step=step)
        if r1_done and r2_done:
            print("[two robot pickup] both arrived at shelf 2")
            break
        sim_tick()
    else:
        print("[two robot pickup] WARNING: did not both arrive within the step budget")
    for _ in range(30):
        sim_tick()
    z2 = obj01.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] Cube_01 z after carry: {z2:.4f} (still on the plates if close to {z1:.4f})")

    print("[two robot pickup] lowering Cube_01 onto shelf 2...")
    global _cargo_check_suspended
    _cargo_check_suspended = True
    while piston_target > piston_rest or piston_target2 > piston_rest2:
        piston_target = max(piston_rest, piston_target - Piston_speed * (1 / 60))
        piston_target2 = max(piston_rest2, piston_target2 - Piston_speed * (1 / 60))
        robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)
        robot2.set_dof_position_targets(piston_target2, dof_indices=piston_joint2)
        sim_tick()
    for _ in range(30):
        sim_tick()
    _cargo_check_suspended = False
    carrying["Robot"] = carrying["Robot2"] = None
    carried_height["Robot"] = carried_height["Robot2"] = None
    z3 = obj01.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] Cube_01 z after lowering: {z3:.4f} (settled if close to {z0:.4f})")

    # WORLD coordinates -- the Property panel shows LOCAL (offset by each
    # robot's own spawn parent transform, y=-0.747 for both here), so these
    # numbers will read ~0.747 higher in the GUI than what's printed here.
    x1, y1, _ = get_xy_yaw(robot)
    x2, y2, _ = get_xy_yaw(robot2)
    print(f"[two robot pickup] final WORLD chassis position -- Robot=({x1:.3f},{y1:.3f}) Robot2=({x2:.3f},{y2:.3f})")

    print("[two robot pickup] all 4 objects delivered -- back to manual control (Robot only)")
    pickup_done = True

_obstacle_count = 0

def spawn_obstacle(r, label):
    """Drops a small physics-enabled cube 0.5m directly ahead of r's
    current position/heading -- scripted instead of relying on a manually
    GUI-created cube so it reliably has real collision every time (a
    RigidBodyAPI + CollisionAPI cube, same convention as the other
    payload objects in the scene) and lands exactly in the robot's path
    regardless of which leg is currently active."""
    global _obstacle_count
    x, y, yaw = get_xy_yaw(r)
    # get_xy_yaw returns numpy scalars -- Gf.Vec3d's binding only accepts
    # native Python floats, not numpy types, hence the ArgumentError.
    ox = float(x + 0.5 * math.cos(yaw))
    oy = float(y + 0.5 * math.sin(yaw))
    path = f"/Obstacle_{_obstacle_count}"
    _obstacle_count += 1
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(ox, oy, 0.15))
    xf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    obstacle_paths.append(path)
    print(f"[two robot pickup] spawned {path} at ({ox:.3f},{oy:.3f}) ahead of {label}")

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World"])
        elif e.input == carb.input.KeyboardInput.P and not pickup_done:
            run_full_delivery()
        elif e.input == carb.input.KeyboardInput.O:
            spawn_obstacle(robot, "Robot")
        elif e.input == carb.input.KeyboardInput.I:
            spawn_obstacle(robot2, "Robot2")
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)
print("[two robot pickup] press P to start the full 4-object delivery run")
print("[two robot pickup] WASD drive Robot, R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame whole map")
print("[two robot pickup] O: drop a physics obstacle ahead of Robot, I: ahead of Robot2")

Tilt_speed = 2
manual_yaw_target = yaw_joint_base
last_time = time.perf_counter()
while simulation_app.is_running():
    now = time.perf_counter()
    dt = now - last_time
    last_time = now

    forward = carb.input.KeyboardInput.W in held_keys
    backward = carb.input.KeyboardInput.S in held_keys
    left = carb.input.KeyboardInput.A in held_keys
    right = carb.input.KeyboardInput.D in held_keys

    speed = 0
    if forward:
        speed += Base_speed
    elif backward:
        speed -= Base_speed

    left_speed = 0
    right_speed = 0
    if left:
        left_speed -= Base_speed
        right_speed += Base_speed
    elif right:
        left_speed += Base_speed
        right_speed -= Base_speed

    robot.set_dof_velocity_targets([speed + left_speed, speed + right_speed, speed + left_speed, speed + right_speed], dof_indices=wheel_joints)

    piston_up = carb.input.KeyboardInput.R in held_keys
    piston_down = carb.input.KeyboardInput.C in held_keys
    roll_left = carb.input.KeyboardInput.LEFT in held_keys
    roll_right = carb.input.KeyboardInput.RIGHT in held_keys
    pitch_up = carb.input.KeyboardInput.UP in held_keys
    pitch_down = carb.input.KeyboardInput.DOWN in held_keys
    yaw_left = carb.input.KeyboardInput.Q in held_keys
    yaw_right = carb.input.KeyboardInput.E in held_keys

    if piston_up:
        piston_target += Piston_speed * dt
    elif piston_down:
        piston_target -= Piston_speed * dt
    piston_target = max(0.0, min(2.0, piston_target))
    robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)

    roll_vel = 0
    if roll_left:
        roll_vel -= Tilt_speed
    elif roll_right:
        roll_vel += Tilt_speed

    pitch_vel = 0
    if pitch_up:
        pitch_vel += Tilt_speed
    elif pitch_down:
        pitch_vel -= Tilt_speed

    robot.set_dof_velocity_targets([roll_vel, pitch_vel], dof_indices=tilt_joints)

    # yaw is now position-controlled (see setup_robot), not velocity --
    # Q/E nudges a manual target the same way R/C nudges the piston.
    if yaw_left:
        manual_yaw_target += Tilt_speed * dt
    elif yaw_right:
        manual_yaw_target -= Tilt_speed * dt
    robot.set_dof_position_targets(manual_yaw_target, dof_indices=yaw_joint)

    simulation_app.update()
    _tick_count += 1
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        d1, _ = min_depth_over(depth_cams1)
        d2, _ = min_depth_over(depth_cams2)
        d1s = f"{d1:.3f}" if d1 is not None else "nothing in view"
        d2s = f"{d2:.3f}" if d2 is not None else "nothing in view"
        print(f"[two robot pickup] depth -- Robot closest: {d1s}  Robot2 closest: {d2s}")

simulation_app.close()
