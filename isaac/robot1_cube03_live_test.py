"""Single-robot (Robot 1 only) interactive test against the dedicated
isolated world (test/robot1_cube03_test.usd) -- Robot plus the real
/Cube_03 obstacle (now with real static collision, see the world-build
session), no Shelf/Shelf2/Robot2. This is the SAME setup, camera, and
navigation code as two_robot_pickup_demo.py, just with every Robot2-
specific piece removed, so the exact real navigation/planning stack gets
exercised against a real, large, known obstacle -- not a throwaway
inline-modified copy of the shelf scene.

Waypoints/constants (DELIVERY_Y, Base_speed, etc.) are kept identical to
the main script's values, per direct instruction -- this is meant to
behave the same as if the shelves/other objects were there, just without
them in the way.

Triggered by pressing P, same convention as the main script."""
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
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, Gf

import occupancy_grid

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "robot1_cube03_test.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[robot1 cube03 test] opened {USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])

art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/Robot/Geometry/base_link"))
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

PLATE_PATH = "actuator_outer_cylinder_link/piston_rod_link/ball_joint_center_link/ball_roll_link/ball_pitch_link/ball_yaw_link/contact_plate_link"
plate1 = RigidPrim(f"/World/Robot/Geometry/base_link/{PLATE_PATH}")

# Same 4-corner camera layout as the main script -- see two_robot_pickup_demo.py
# for the full derivation/rationale, unchanged here.
CAMERA_HEIGHT = 0.10
CAMERA_CORNERS = [
    ("FL", np.array([0.15, 0.075, CAMERA_HEIGHT]), 45.0),
    ("FR", np.array([0.15, -0.075, CAMERA_HEIGHT]), -45.0),
    ("RL", np.array([-0.15, 0.075, CAMERA_HEIGHT]), 135.0),
    ("RR", np.array([-0.15, -0.075, CAMERA_HEIGHT]), -135.0),
]

def _yaw_quat(deg):
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
    cam.set_clipping_range(near_distance=0.02, far_distance=50.0)
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

def min_depth_over(cams):
    best_dist, best_cam = None, None
    for cam in cams:
        d = min_depth(cam)
        if d is not None and (best_dist is None or d < best_dist):
            best_dist, best_cam = d, cam
    return best_dist, best_cam

DEPTH_FRAME_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "depth_frames")
MAX_DEPTH_RUNS = 3
DEPTH_VIS_MAX = 3.0

def _prepare_depth_frame_run_dir():
    os.makedirs(DEPTH_FRAME_ROOT, exist_ok=True)
    existing = sorted(
        d for d in os.listdir(DEPTH_FRAME_ROOT)
        if d.startswith("run_") and os.path.isdir(os.path.join(DEPTH_FRAME_ROOT, d))
    )
    for old in existing[: max(0, len(existing) - (MAX_DEPTH_RUNS - 1))]:
        shutil.rmtree(os.path.join(DEPTH_FRAME_ROOT, old))
        print(f"[robot1 cube03 test] pruned old depth-frame run: {old}")
    run_dir = os.path.join(DEPTH_FRAME_ROOT, "run_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    print(f"[robot1 cube03 test] depth frames for this run -> {run_dir}")
    return run_dir

DEPTH_FRAME_DIR = _prepare_depth_frame_run_dir()

def dump_depth_frame(cam, tag):
    d = raw_depth_frame(cam)
    if d is None:
        return None
    d = np.asarray(d, dtype=float)
    d = np.where(np.isfinite(d), d, DEPTH_VIS_MAX)
    d = np.clip(d, 0.0, DEPTH_VIS_MAX)
    safe_tag = re.sub(r"[^A-Za-z0-9_.-]+", "_", tag)
    path = os.path.join(DEPTH_FRAME_DIR, f"{safe_tag}_{_tick_count:06d}.png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(path, DEPTH_VIS_MAX - d, cmap="inferno", vmin=0.0, vmax=DEPTH_VIS_MAX)
    return path

def dump_grid_frame(path, tag):
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
    plt.imsave(frame_path, np.flipud(img))
    return frame_path

def raw_depth_frame(cam):
    frame = cam.get_current_frame()
    return frame.get("distance_to_camera") if frame else None

def min_depth(cam):
    d = raw_depth_frame(cam)
    if d is None:
        return None
    finite = d[np.isfinite(d)]
    return float(finite.min()) if finite.size else None

def identify_obstacle(cam):
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
    print(f"[robot1 cube03 test] identify_obstacle: cam pos={pos} fwd={fwd} raw hit dict={hit}")
    if hit and hit.get("rigidBody"):
        return hit["rigidBody"], hit["distance"]
    return None, None

nav_grid = occupancy_grid.OccupancyGrid()

DELIVERY_YAW = math.pi / 2
DELIVERY_Y = 3.25
Base_speed = 5.0
Piston_speed = 0.15
LIFT_TARGET = 0.18
pickup_done = False

carrying = {"Robot": None}
carried_height = {"Robot": None}
CARGO_DROP_TOLERANCE = 0.08
_cargo_check_suspended = False

def check_cargo():
    if _cargo_check_suspended:
        return
    for label, obj_path in list(carrying.items()):
        if obj_path is None:
            continue
        expected_z = carried_height[label]
        z = float(RigidPrim(obj_path).get_world_poses()[0].numpy()[0, 2])
        if abs(z - expected_z) > CARGO_DROP_TOLERANCE:
            print(f"[robot1 cube03 test] {label} WARNING: cargo lost (ground-truth check) -- {obj_path} z={z:.4f}, expected ~{expected_z:.4f}")
            carrying[label] = None
            carried_height[label] = None

OBSTACLE_THRESHOLD = 0.4
STALL_ABORT_TICKS = 300

def get_xy_yaw(r):
    pos, quat = r.get_world_poses()
    p = pos.numpy()[0]
    w, x, y, z = quat.numpy()[0]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return p[0], p[1], yaw

RAMP_STEPS = 90
DECEL_DISTANCE = 0.3
MIN_RAMP_OUT = 0.6  # see two_robot_pickup_demo.py's comment on this same
                     # constant -- raised from 0.15 this session after
                     # isolating a genuine low-commanded-speed wheel-
                     # tracking instability, not a "ramp too weak" issue.
DECEL_ANGLE = math.radians(15)

def turn_to_yaw_step(r, wj, target_yaw, step=RAMP_STEPS, speed=Base_speed, yaw_tol=0.002):
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
    ramp_in = min(1.0, step / RAMP_STEPS)
    x, y, yaw = get_xy_yaw(r)
    remaining = (target_x - x) * math.cos(yaw) + (target_y - y) * math.sin(yaw)
    if remaining <= stop_dist:
        r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
        return True
    ramp_out = max(MIN_RAMP_OUT, min(1.0, remaining / DECEL_DISTANCE))
    speed = speed * min(ramp_in, ramp_out)
    r.set_dof_velocity_targets([speed] * 4, dof_indices=wj)
    return False

SETTLE_FRAMES = 60

def apply_yaw_compensation(r, yj, yaw_base, chassis_yaw0):
    _, _, chassis_yaw = get_xy_yaw(r)
    delta = math.atan2(math.sin(chassis_yaw - chassis_yaw0), math.cos(chassis_yaw - chassis_yaw0))
    r.set_dof_position_targets(yaw_base - delta, dof_indices=yj)

_, _, chassis_yaw0 = get_xy_yaw(robot)

_tick_count = 0
DEPTH_PRINT_INTERVAL = 60
TRACE_INTERVAL = 600

KNOWN_OBJECTS = []  # Cube_03 is a fixed static obstacle, not a carryable
                     # payload and not moving -- no need to poll its pose
                     # every tick, its real position is already known
                     # (queried when the isolated world was built).
obstacle_paths = []  # appended by spawn_obstacle -- these DO have
                      # RigidBodyAPI, unlike Cube_03, so tracking them here
                      # (via RigidPrim) is safe.

def print_trace():
    x, y, yaw = get_xy_yaw(robot)
    print(f"[robot1 cube03 test] TRACE Robot: pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f}")
    for path in KNOWN_OBJECTS + obstacle_paths:
        prim = RigidPrim(path)
        pos = prim.get_world_poses()[0].numpy()[0]
        _, _, oyaw = get_xy_yaw(prim)
        held_by = next((lbl for lbl, p in carrying.items() if p == path), None)
        tag = f" (on {held_by}'s plate)" if held_by else ""
        print(f"[robot1 cube03 test] TRACE {path}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) yaw={math.degrees(oyaw):.1f}{tag}")

def sim_tick():
    global _tick_count
    apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
    simulation_app.update()
    check_cargo()
    for cam in depth_cams1:
        occupancy_grid.update_from_camera(nav_grid, cam, raw_depth_frame(cam))
    _tick_count += 1
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        d1, _ = min_depth_over(depth_cams1)
        d1s = f"{d1:.3f}" if d1 is not None else "nothing in view"
        print(f"[robot1 cube03 test] depth -- Robot closest: {d1s}")
    if _tick_count % TRACE_INTERVAL == 0:
        print_trace()
        for i, cam in enumerate(depth_cams1):
            dump_depth_frame(cam, f"Robot_periodic_{i}")

def run_turn(r, wj, target_yaw, label="", stall_abort_ticks=None, cam=None):
    this_yj = yaw_joint if r is robot else None
    if this_yj is not None:
        r.set_dof_gains(stiffnesses=0.0, dampings=0.0, dof_indices=this_yj)
    start_tick = _tick_count
    start_yaw = get_xy_yaw(r)[2]
    print(f"[robot1 cube03 test] {label} TASK START rotate tick={start_tick} from={math.degrees(start_yaw):.1f} to={math.degrees(target_yaw):.1f} deg")
    prev_yaw = start_yaw
    stall_count = 0
    finished = False
    abort_reason = None
    max_wheel_track_err = 0.0
    for step in range(60000):  # generous, not a real timeout -- for live
                                 # viewing, the run shouldn't get cut off
                                 # just because it took a while while
                                 # someone's watching. Real termination is
                                 # still "reached target" or genuine stall
                                 # (stall_abort_ticks), this is just a
                                 # last-resort safety cap.
        _, _, yaw_now = get_xy_yaw(r)
        d_yaw = math.atan2(math.sin(yaw_now - prev_yaw), math.cos(yaw_now - prev_yaw))
        stall_count = stall_count + 1 if (abs(d_yaw) < 1e-5 and step > RAMP_STEPS) else 0
        if step % 30 == 0:
            actual_vel = r.get_dof_velocities(dof_indices=wj).numpy()[0]
            target_vel = r.get_dof_velocity_targets(dof_indices=wj).numpy()[0]
            track_err = float(np.max(np.abs(actual_vel - target_vel)))
            max_wheel_track_err = max(max_wheel_track_err, track_err)
            # Live diagnostic -- what it's doing and what it currently sees,
            # printed continuously (every ~0.5s), not just on a stall or a
            # hard obstacle stop, so a human watching live can follow along.
            d_live, _ = min_depth_over(cam) if cam is not None else (None, None)
            d_live_s = f"{d_live:.3f}m" if d_live is not None else "nothing in view"
            print(f"[robot1 cube03 test] {label} LIVE turning: yaw={math.degrees(yaw_now):.1f} -> target={math.degrees(target_yaw):.1f} deg, nearest depth={d_live_s}")
        if stall_count and stall_count % 200 == 0:
            actual_vel = r.get_dof_velocities(dof_indices=wj).numpy()[0]
            target_vel = r.get_dof_velocity_targets(dof_indices=wj).numpy()[0]
            print(f"[robot1 cube03 test] {label} STALL WARNING at step={step}: yaw={math.degrees(yaw_now):.2f} not changing (stuck {stall_count} ticks) -- wheels actual={actual_vel} target={target_vel}")
            px, py, pz = r.get_world_poses()[0].numpy()[0]
            print(f"[robot1 cube03 test] {label} chassis position check -- pos=({px:.3f},{py:.3f},{pz:.3f})")
            if cam is not None:
                print(f"[robot1 cube03 test] {label} stall carrying-state -- {carrying}")
                for i, one_cam in enumerate(cam):
                    hit_body, hit_dist = identify_obstacle(one_cam)
                    d = min_depth(one_cam)
                    frame_path = dump_depth_frame(one_cam, f"{label.replace(' ', '_')}_stallcam{i}")
                    ds = f"{d:.3f}" if d is not None else "nothing in view"
                    print(f"[robot1 cube03 test] {label} stall cam#{i} -- raycast touching: {hit_body} (dist={hit_dist}), min_depth={ds} [frame: {frame_path}]")
        if stall_abort_ticks is not None and stall_count >= stall_abort_ticks:
            print(f"[robot1 cube03 test] {label} ABORTING turn -- stalled for {stall_count} ticks")
            r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
            abort_reason = "stalled"
            break
        prev_yaw = yaw_now
        if turn_to_yaw_step(r, wj, target_yaw, step=step):
            finished = True
            break
        sim_tick()
    else:
        print(f"[robot1 cube03 test] WARNING: {label} did not finish turning within the step budget")
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
    print(f"[robot1 cube03 test] {label} TASK END rotate result={result_str} duration_ticks={duration_ticks} final_yaw_err_deg={final_err_deg:.3f} tolerance_deg={math.degrees(0.002):.3f} max_wheel_track_err={max_wheel_track_err:.3f} reason: {reason_str}")
    for _ in range(SETTLE_FRAMES):
        sim_tick()
    return finished

def run_drive(r, wj, target_x, target_y, label="", cam=None, cargo_key=None):
    print(f"[robot1 cube03 test] {label} driving to ({target_x:.3f},{target_y:.3f})...")
    for step in range(60000):  # generous, not a real timeout -- see run_turn
        if step % 30 == 0:
            # Live diagnostic every ~0.5s -- what it's doing and what it
            # currently sees, even when nothing is close enough to block.
            x, y, yaw = get_xy_yaw(r)
            dist = math.hypot(target_x - x, target_y - y)
            d_live, _ = min_depth_over(cam) if cam is not None else (None, None)
            d_live_s = f"{d_live:.3f}m" if d_live is not None else "nothing in view"
            print(f"[robot1 cube03 test] {label} LIVE driving: pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f} remaining={dist:.3f}m, nearest depth={d_live_s}")
        if cargo_key is not None and carrying.get(cargo_key) is None:
            r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
            print(f"[robot1 cube03 test] {label} stopping -- cargo already lost")
            return "cargo_lost"
        if cam is not None:
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
                print(f"[robot1 cube03 test] {label} obstacle at {d:.3f}m ahead -- stopping (raycast hit: {hit_body}, dist={hit_dist}) [depth frame: {frame_path}]")
                return "blocked"
        if drive_to_point_step(r, wj, target_x, target_y, step=step):
            for _ in range(30):
                sim_tick()
            return "arrived"
        sim_tick()
    print(f"[robot1 cube03 test] WARNING: {label} did not arrive within the step budget")
    for _ in range(30):
        sim_tick()
    return "timeout"

OBJECT_HALF_EXTENTS = {}  # no carryable payload objects in this isolated
                           # world -- current_rotation_radius falls back to
                           # chassis-only radius for any unknown path.

def current_rotation_radius(label):
    obj_path = carrying.get(label)
    if obj_path is None:
        return occupancy_grid.rotation_swept_radius()
    half_x, half_y = OBJECT_HALF_EXTENTS.get(obj_path, (0.0, 0.0))
    return occupancy_grid.rotation_swept_radius(half_x, half_y)

MAX_PLAN_ATTEMPTS = 30

def execute_path(r, wj, path, label, cam, cargo_key):
    for wp_x, wp_y in path[1:]:
        x0, y0, _ = get_xy_yaw(r)
        bearing = bearing_to(x0, y0, wp_x, wp_y)
        turned = run_turn(r, wj, bearing, label=label, stall_abort_ticks=STALL_ABORT_TICKS, cam=cam)
        if not turned:
            return "blocked"
        result = run_drive(r, wj, wp_x, wp_y, label=label, cam=cam, cargo_key=cargo_key)
        if result == "cargo_lost":
            return "cargo_lost"
        if result != "arrived":
            return "blocked"
        x1, y1, _ = get_xy_yaw(r)
        if math.hypot(x1 - x0, y1 - y0) < 0.02:
            return "no_progress"
    return "arrived"

def run_goto(r, wj, target_x, target_y, label="", cam=None, cargo_key=None):
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
        grid_frame_path = dump_grid_frame(path, label.replace(" ", "_") + f"_attempt{attempt}")
        if path is None or len(path) < 2:
            print(f"[robot1 cube03 test] {label} WARNING: no path to target found -- attempting minimal-displacement recovery instead [grid frame: {grid_frame_path}]")
            result = "blocked"
        else:
            print(f"[robot1 cube03 test] {label} planned {len(path)}-waypoint path [grid frame: {grid_frame_path}]")
            result = execute_path(r, wj, path, label, cam, cargo_key)
            if result == "arrived":
                return True
            if result == "cargo_lost":
                return False

        print(f"[robot1 cube03 test] {label} {result} -- planning minimal-displacement recovery to nearest clear cell...")
        x, y, _ = get_xy_yaw(r)
        clear_path = occupancy_grid.plan_to_clear(nav_grid, (x, y), radius)
        if clear_path is None:
            print(f"[robot1 cube03 test] {label} WARNING: no reachable clear cell found -- genuinely stuck, giving up")
            return False
        if len(clear_path) < 2:
            # plan_to_clear returned just [start_xy] -- the grid itself
            # already considers the current cell clear (not occupied+
            # inflated). The "blocked" trigger came from the real-time
            # reactive sensor threshold, not the grid, so this is a
            # timing mismatch, not an actual dead end -- nothing to move,
            # just loop back and retry the main plan instead of treating
            # it as the same failure as a real exhausted search.
            print(f"[robot1 cube03 test] {label} already clear per the grid -- retrying main plan")
        else:
            recovery_result = execute_path(r, wj, clear_path, f"{label} recovery", cam, cargo_key)
            if recovery_result == "cargo_lost":
                return False
    print(f"[robot1 cube03 test] {label} WARNING: still not at target after {MAX_PLAN_ATTEMPTS} replans, giving up")
    return False

_obstacle_count = 0

def spawn_obstacle(r, label):
    global _obstacle_count
    x, y, yaw = get_xy_yaw(r)
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
    print(f"[robot1 cube03 test] spawned {path} at ({ox:.3f},{oy:.3f}) ahead of {label}")

def _robot_collision_prim_paths():
    """Every prim under /World/Robot that actually participates in
    collision (RigidBodyAPI or CollisionAPI applied) -- the robot is a
    multi-body articulation (chassis, wheels, actuator/plate links all
    separate rigid bodies), so a single filter target isn't enough to
    exclude the whole robot."""
    return [
        prim.GetPath()
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"))
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]

def spawn_waypoint_marker(r, label):
    """Drops a visual "go here" marker 0.5m ahead of r's current heading --
    real collision + RigidBodyAPI (gravity applies, it falls and actually
    lands/rests on the ground, unlike the first version of this which had
    no collision at all and just sank through the floor), but filtered
    (UsdPhysics.FilteredPairsAPI) against every collision-bearing prim
    under Robot specifically, so it never physically touches or blocks
    the robot -- collides with the ground, passes through Robot."""
    global _obstacle_count
    x, y, yaw = get_xy_yaw(r)
    ox = float(x + 0.5 * math.cos(yaw))
    oy = float(y + 0.5 * math.sin(yaw))
    path = f"/Waypoint_{_obstacle_count}"
    _obstacle_count += 1
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(ox, oy, 0.5))
    xf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    filt = UsdPhysics.FilteredPairsAPI.Apply(prim)
    filt.CreateFilteredPairsRel().SetTargets(_robot_collision_prim_paths())
    obstacle_paths.append(path)
    print(f"[robot1 cube03 test] spawned waypoint marker {path} at ({ox:.3f},{oy:.3f}) ahead of {label} -- collides with the ground (will land/rest), filtered out of collision with Robot")

# Real position, queried directly from robot1_cube03_test.usd when the
# isolated world was built -- not guessed.
CUBE03_X, CUBE03_Y = 0.724883990193226, 1.3277432645275122
_spawn_x, _spawn_y, _ = get_xy_yaw(robot)

def run_obstacle_test():
    """Bound to P. Drives Robot from its spawn position straight through
    Cube_03's real center, extended out to DELIVERY_Y -- forces the
    planner to actually route around the real obstacle, using the exact
    same navigation stack (run_goto/plan_path/plan_to_clear) as the main
    two-robot script's carry legs."""
    global pickup_done
    dx, dy = CUBE03_X - _spawn_x, CUBE03_Y - _spawn_y
    t = (DELIVERY_Y - _spawn_y) / dy
    target_x, target_y = _spawn_x + t * dx, DELIVERY_Y
    print(f"[robot1 cube03 test] target ({target_x:.3f},{target_y:.3f}) -- straight line from spawn through Cube_03's real center")
    arrived = run_goto(robot, wheel_joints, target_x, target_y, label="Robot", cam=depth_cams1)
    x, y, yaw = get_xy_yaw(robot)
    print(f"[robot1 cube03 test] DONE -- arrived={arrived} final pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f}")
    pickup_done = True

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])
        elif e.input == carb.input.KeyboardInput.P and not pickup_done:
            run_obstacle_test()
        elif e.input == carb.input.KeyboardInput.O:
            spawn_waypoint_marker(robot, "Robot")
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)
print("[robot1 cube03 test] press P to drive toward/through Cube_03 (real planner, real obstacle)")
print("[robot1 cube03 test] WASD drive Robot, R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame the scene")
print("[robot1 cube03 test] O: drop a waypoint marker (no collision, falls under gravity) ahead of Robot")

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

    if yaw_left:
        manual_yaw_target += Tilt_speed * dt
    elif yaw_right:
        manual_yaw_target -= Tilt_speed * dt
    robot.set_dof_position_targets(manual_yaw_target, dof_indices=yaw_joint)

    simulation_app.update()
    _tick_count += 1
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        d1, _ = min_depth_over(depth_cams1)
        d1s = f"{d1:.3f}" if d1 is not None else "nothing in view"
        print(f"[robot1 cube03 test] depth -- Robot closest: {d1s}")
