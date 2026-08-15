"""Robot 1 + Nav2 bridge, via Isaac Sim's native OmniGraph ROS2 nodes --
NOT Python rclpy. rclpy-in-Kit-process was tried first and hit a hard,
unfixable-via-env-vars crash this session: the compiled ROS2 message
bindings (rosidl_generator_py) are built against Python 3.10 (ROS2
Humble's standard target), but Isaac Sim 6.0's Kit runs Python 3.12 --
confirmed via a `PyUnicode_IS_READY` assertion failure (a macro Python
3.12 removed as a real check), reproduced identically whether using
system ROS2 Humble OR Isaac's own bundled "internal rclpy for humble".
OmniGraph's ROS2 nodes are native C++ with their own DDS bindings, so
they never cross that broken Python/C boundary at all.

Same physical robot/camera/piston setup as robot1_cube03_live_test.py
(same isolated world, same real chassis/wheel geometry) -- only the
navigation layer changes. WASD wheel control is dropped here (cmd_vel,
driven by Nav2 once launched separately via `ros2 launch nav2_bringup
navigation_launch.py`, now owns the wheels); piston/plate manual control
(R/C, arrows, Q/E) is unchanged.

Launch with the ROS2 env vars that caused the Python-version conflict
cleared (PYTHONPATH/AMENT_PREFIX_PATH/COLCON_PREFIX_PATH), but ROS_DISTRO
left set -- Isaac's own isaacsim.ros2.core extension requires it even
though this script never imports rclpy:

    PYTHONPATH= AMENT_PREFIX_PATH= COLCON_PREFIX_PATH= ./python.sh nav2_bridge_robot1.py
"""
from isaacsim import SimulationApp
import os as _os_early  # needed before other imports, just for the
                         # headless env-var check below

simulation_app = SimulationApp(launch_config={
    # Overridable via env var (HEADLESS=1) rather than a hard default
    # change, so GUI mode is still one env var away for live-watched
    # work. Now that real automated checks exist (cargo attachment,
    # plate orientation) instead of relying on screenshots, headless
    # verification runs are viable per user request this session.
    "headless": _os_early.environ.get("HEADLESS", "0") == "1",
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
    "width": 960,
    "height": 540,
    "window_width": 1000,
    "window_height": 640,
})

import math
import os
import time

import numpy as np
import carb
import carb.settings
import omni.timeline
import omni.appwindow
import carb.input
import omni.graph.core as og
import usdrt.Sdf
import isaacsim.core.experimental.utils.app as app_utils
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from omni.physx import get_physx_scene_query_interface
from isaacsim.sensors.camera import Camera
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, Gf

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

# Enable the ROS2 bridge BEFORE the graph is built -- matches every real
# Isaac Sim ROS2 example (subscriber.py, carter_multiple_robot_navigation.py).
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# two_robot_two_shelf.usd -- the original, full scene (both shelves,
# Robot2, real collidable payload objects), not the minimal single-robot
# scene used earlier this session for navigation-only testing. Robot2 is
# present in the file but deliberately never set up/driven below, so it
# just sits inert -- "disabled" per user request, not stripped from the
# USD.
USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "two_robot_two_shelf.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[nav2 bridge robot1] opened {USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])

art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/Robot/Geometry/base_link"))
art_api.CreateSolverPositionIterationCountAttr(255)
art_api.CreateSolverVelocityIterationCountAttr(64)

# Isaac's Articulation physics-tensor API (get_dof_positions/get_world_poses,
# used throughout setup below) is only valid once the timeline has played at
# least once -- confirmed live this session: skipping this entirely raises
# "Instance's physics tensor entity is not valid. Play the simulation/
# timeline to re-initialize it". So play briefly to initialize it, then
# pause again below once setup is done -- the window comes up paused and
# ready, not auto-running, but setup still works.
omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

ROBOT_PATH = "/World/Robot/Geometry/base_link"

# Overridable via env var so a boot-time sweep doesn't need a code edit
# each time, e.g. `WHEEL_DOF_DAMPING=50 ./python.sh nav2_bridge_robot1.py`.
# Reverted to the original 5000 -- live-tested this session and traced
# the earlier stall to PhysX articulation sleep (see set_sleep_thresholds
# below), not this gain; a lower damping (5.0) was tested directly and
# made things strictly worse (even less responsive), so 5000 stands.
WHEEL_DOF_DAMPING = float(os.environ.get("WHEEL_DOF_DAMPING", "5000.0"))

def setup_robot(articulation_path):
    r = Articulation(articulation_path)
    # PhysX puts stationary articulations to sleep after sitting under
    # its sleep-threshold velocity for a while -- confirmed this session:
    # the chassis responded normally to a raw /cmd_vel command right
    # after boot, then stopped responding at all (regardless of wheel
    # damping/effort tuning) after several minutes of live testing with
    # the chassis mostly at rest, which only makes sense as a sleep
    # state, not a gain problem. Threshold ~0 means it practically never
    # qualifies as "at rest" and stays awake.
    r.set_sleep_thresholds(0.0)
    r.set_link_masses([40.0], link_indices=r.get_link_indices(["base_link"]))
    wj = r.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
    # damping=5000 here originally -- live-tested this session and found
    # it unstable at the small commands Nav2 sends: a wheel this light
    # has tiny rotational inertia, so a damping gain that huge relative
    # to it causes the drive torque to overshoot and flip sign every
    # physics substep instead of converging (classic explicit-integrator
    # instability when gain is disproportionate to inertia) -- confirmed
    # by testing a max-effort cap up to 50 N*m with zero improvement,
    # which rules out "not enough torque" and points at the gain itself.
    # WHEEL_DOF_DAMPING is set way down below at the module level once a
    # live-tuning sweep (via /tmp/robot1_damping) finds a stable value.
    r.set_dof_gains(stiffnesses=0, dampings=WHEEL_DOF_DAMPING, dof_indices=wj)
    r.set_dof_velocity_targets(0, dof_indices=wj)
    rp = r.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=rp)
    rp_base = r.get_dof_positions(dof_indices=rp).numpy()[0].tolist()
    r.set_dof_position_targets(rp_base, dof_indices=rp)
    yj = r.get_dof_indices(["ball_yaw_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=yj)
    # Asset default is maxForce=200 (same as the piston's ORIGINAL,
    # confirmed-too-low value before that was raised 100->200 through live
    # testing under real load). This joint has never been torque-tested
    # under rapid successive turns while carrying weight -- live-caught
    # this session: real orientation drift up to ~77deg from its target
    # during a multi-turn detour route, meaning the PD drive (stiffness
    # 5000, damping 200) can't accelerate the joint+carried-load inertia
    # fast enough to track a quickly-moving target, capped by this force
    # budget. Raised here as a live runtime override (not editing the
    # shared asset file) so it's easy to re-tune/revert -- same kind of
    # empirical raise as the piston's, not yet confirmed as the right
    # final value.
    r.set_dof_max_efforts(1000.0, dof_indices=yj)
    yaw_joint_pos = float(r.get_dof_positions(dof_indices=yj).numpy()[0, 0])
    r.set_dof_position_targets(yaw_joint_pos, dof_indices=yj)
    pj = r.get_dof_indices(["centered_piston_prismatic_z"])
    r.set_dof_limits(lower=0.0, upper=2.0, dof_indices=pj)
    r.set_dof_gains(stiffnesses=20000.0, dampings=1000.0, dof_indices=pj)
    piston_pos = float(r.get_dof_positions(dof_indices=pj).numpy()[0, 0])
    r.set_dof_position_targets(piston_pos, dof_indices=pj)
    return r, wj, rp, yj, yaw_joint_pos, pj, piston_pos

robot, wheel_joints, tilt_joints, yaw_joint, yaw_joint_base, piston_joint, piston_target = setup_robot(ROBOT_PATH)

# Robot2 is deliberately never driven (no camera, no yaw compensation, no
# navigation) -- "disabled" per user request. But its USD default wheel
# joint state carries a nonzero velocity target from whenever
# two_robot_pickup_demo.py last drove it and the scene was saved --
# confirmed live this session: left completely untouched, it slides at
# ~0.65 m/s indefinitely. setup_robot() zeroes drive targets/sets real
# gains for every joint (same as Robot1's own setup below) -- calling it
# here only parks Robot2 in place, it does not add any active control.
ROBOT2_PATH = "/World/Robot2/Geometry/base_link"
setup_robot(ROBOT2_PATH)

# Pickup/carry constants ported directly from two_robot_pickup_demo.py --
# real, live-tuned values from that script's proven pickup sequence, not
# re-derived here. PISTON_REST is this robot's own real starting piston
# position (read from the joint at setup, not a guessed constant).
PISTON_REST = piston_target
LIFT_TARGET = 0.18
CARGO_DROP_TOLERANCE = 0.08  # meters, matches two_robot_pickup_demo.py
CARGO_STATUS_FILE = "/tmp/robot1_cargo_status"  # "attached:<path>" / "none"
                                                  # / "dropped" -- written
                                                  # whenever this changes,
                                                  # so an external
                                                  # orchestrator can check
                                                  # real attachment instead
                                                  # of trusting an open-loop
                                                  # lift sequence completed.
carrying_path = None    # prim path of whatever's currently on the plate,
carrying_offset = None  # (dx, dy, dz) from chassis at the moment it was
                        # lifted, or None if nothing -- ground truth, not
                        # inferred. Checking the object's position
                        # RELATIVE to the moving chassis, not just its
                        # absolute Z, since a sideways knock (e.g. from
                        # clipping shelf structure) can push cargo off
                        # the plate without dropping much in height --
                        # confirmed live this session: a real fall-off
                        # went undetected by a Z-only check.

# Same 4-corner depth camera layout as robot1_cube03_live_test.py -- kept
# for later LaserScan publishing (task #19), unused by the graph so far.
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

depth_cams1 = add_corner_cameras(ROBOT_PATH)

def raw_depth_frame(cam):
    frame = cam.get_current_frame()
    return frame.get("distance_to_camera") if frame else None

# LaserScan synthesis -- reuses the exact same per-camera ray-angle
# geometry as occupancy_grid.py's update_from_camera (same FOV, same
# near-center row band), just reorganized into a uniform 360-degree scan
# for ROS2PublishLaserScan's plain `linearDepthData: float[]` input,
# which is fully decoupled from needing an actual RTX Lidar sensor --
# confirmed via its .ogn definition. This stays entirely on the
# OmniGraph/Kit side (og.Controller.set, not rclpy), so it doesn't touch
# the broken Python/ROS2 boundary at all.
SCAN_HORIZONTAL_FOV_DEG = 90.0  # matches set_focal_length(10.5) above
SCAN_ROW_HALF_BAND = 1          # near-single-row sample, see
                                  # occupancy_grid.py's own comment for
                                  # why a wider band produces phantom hits
SCAN_MAX_RANGE = 3.0             # matches DEPTH_VIS_MAX/MAX_RANGE used
                                  # elsewhere this session for this robot
LASER_NUM_SAMPLES = 128          # 4 cameras x 32 columns each -- same
                                  # resolution occupancy_grid.py samples
LASER_ANGLE_MIN = -math.pi
LASER_ANGLE_MAX = math.pi
LASER_ANGLE_INC = (LASER_ANGLE_MAX - LASER_ANGLE_MIN) / LASER_NUM_SAMPLES

def _sample_camera_range(cam, local_angle_in_cam_frame):
    """local_angle_in_cam_frame: angle relative to the camera's OWN
    forward direction. Returns the range at that angle from the camera's
    current depth frame, or None if the angle is outside this camera's
    FOV (caller tries the next camera), or SCAN_MAX_RANGE if in-FOV but
    nothing finite was seen."""
    half_fov = math.radians(SCAN_HORIZONTAL_FOV_DEG) / 2.0
    if abs(local_angle_in_cam_frame) > half_fov:
        return None
    frame = raw_depth_frame(cam)
    if frame is None:
        return SCAN_MAX_RANGE
    h, w = frame.shape
    row_center = h // 2
    row_lo = max(0, row_center - SCAN_ROW_HALF_BAND)
    row_hi = min(h, row_center + SCAN_ROW_HALF_BAND + 1)
    c = int(round(((local_angle_in_cam_frame / half_fov) / 2.0 + 0.5) * (w - 1)))
    c = max(0, min(w - 1, c))
    col_vals = frame[row_lo:row_hi, c]
    finite = col_vals[np.isfinite(col_vals)]
    return min(float(finite.min()), SCAN_MAX_RANGE) if finite.size else SCAN_MAX_RANGE

CAMERA_OVERLAP_RECONCILE_TOL = 0.1  # meters -- the 4 corner cameras are
                                  # separate physical viewpoints, so where
                                  # their 90deg FOVs overlap they see the
                                  # same real surface at different parallax,
                                  # not a genuine disagreement. If two
                                  # cameras' readings at the same body-frame
                                  # angle are within this of each other,
                                  # average them into one consistent value
                                  # instead of the old naive min() (which
                                  # could flip abruptly between cameras
                                  # sample to sample at the seam, producing
                                  # a jagged boundary for one real smooth
                                  # surface -- "layering" that fed spurious
                                  # occupied cells into the costmap/planner).
                                  # A genuine disagreement beyond this
                                  # tolerance (e.g. one camera sees past an
                                  # edge the other can't) still keeps the
                                  # closer/more conservative reading.
SMOOTH_MEDIAN_WINDOW = 3         # odd window (samples either side of each
                                  # angle) -- knocks out single-sample
                                  # seam/edge-grazing noise spikes before
                                  # they reach the costmap.

def _median_smooth(values, window):
    """Circular median filter -- this is a full 360deg scan, so angle
    index 0 and index -1 are physically adjacent, not a hard edge."""
    half = window // 2
    n = len(values)
    out = []
    for i in range(n):
        neighborhood = sorted(values[(i + k) % n] for k in range(-half, half + 1))
        out.append(neighborhood[len(neighborhood) // 2])
    return out

def synthesize_laser_ranges(cams):
    """One range per fixed robot-frame angle (LASER_NUM_SAMPLES evenly
    spaced from -pi to pi, matching ROS2PublishLaserScan's azimuthRange
    [-180,180]). Where more than one corner camera's FOV covers a given
    direction, reconciles their readings (see CAMERA_OVERLAP_RECONCILE_TOL)
    instead of naively taking whichever is smaller. Returned in increasing-
    azimuth order, as the node requires."""
    ranges = []
    for i in range(LASER_NUM_SAMPLES):
        angle = LASER_ANGLE_MIN + i * LASER_ANGLE_INC
        hits = []
        for cam, (_, _, cam_yaw_deg) in zip(cams, CAMERA_CORNERS):
            local_angle = angle - math.radians(cam_yaw_deg)
            local_angle = math.atan2(math.sin(local_angle), math.cos(local_angle))
            r = _sample_camera_range(cam, local_angle)
            if r is not None:
                hits.append(r)
        if not hits:
            ranges.append(SCAN_MAX_RANGE)
        elif len(hits) == 1:
            ranges.append(hits[0])
        else:
            lo, hi = min(hits), max(hits)
            if hi - lo <= CAMERA_OVERLAP_RECONCILE_TOL:
                ranges.append(sum(hits) / len(hits))
            else:
                ranges.append(lo)
    return _median_smooth(ranges, SMOOTH_MEDIAN_WINDOW)

def get_xy_yaw(r):
    pos, quat = r.get_world_poses()
    p = pos.numpy()[0]
    w, x, y, z = quat.numpy()[0]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return p[0], p[1], yaw

def apply_yaw_compensation(r, yj, yaw_base, chassis_yaw0):
    _, _, chassis_yaw = get_xy_yaw(r)
    delta = math.atan2(math.sin(chassis_yaw - chassis_yaw0), math.cos(chassis_yaw - chassis_yaw0))
    r.set_dof_position_targets(yaw_base - delta, dof_indices=yj)

def raycast_from(pos, dir_xy, max_dist=1.0, start_offset=0.2):
    """Ground-truth PhysX raycast -- same pattern as
    two_robot_pickup_demo.py's identify_obstacle(), reused directly
    rather than re-derived. start_offset pushes the ray's actual start
    point out from pos by this much first, so it doesn't immediately
    self-hit whatever body pos is inside/on the surface of (confirmed
    live this session: without this, every direction hit distance 0.0
    against the robot's/cargo's own collision geometry). Returned
    distance is measured from the ORIGINAL pos, not the offset start, so
    it's directly comparable to a real-world clearance number. Returns
    (hit_body_path, distance) or (None, None)."""
    dx, dy = dir_xy
    mag = math.hypot(dx, dy) or 1.0
    ux, uy = dx / mag, dy / mag
    start = (pos[0] + ux * start_offset, pos[1] + uy * start_offset, pos[2])
    hit = get_physx_scene_query_interface().raycast_closest(
        carb.Float3(float(start[0]), float(start[1]), float(start[2])),
        carb.Float3(float(ux), float(uy), 0.0),
        max(0.01, max_dist - start_offset),
        bothSides=True,
    )
    if hit and hit.get("rigidBody"):
        return hit["rigidBody"], hit["distance"] + start_offset
    return None, None

_, _, chassis_yaw0 = get_xy_yaw(robot)

# Real-world plate-orientation check, added after live-catching a bug
# this session where apply_yaw_compensation was computing the right
# counter-rotation target every tick but a leftover manual-keyboard-
# control code path was silently overwriting it on the same joint a few
# lines later -- the compensation never actually reached the joint, and
# nothing was checking, so it went undetected until a carried object
# visibly swung into other objects. This checks the plate's REAL world
# orientation (not just whether the right joint target was set) against
# its own orientation at startup, continuously.
PLATE_PATH = "actuator_outer_cylinder_link/piston_rod_link/ball_joint_center_link/ball_roll_link/ball_pitch_link/ball_yaw_link/contact_plate_link"
plate = RigidPrim(f"{ROBOT_PATH}/{PLATE_PATH}")
_, _, plate_yaw0 = get_xy_yaw(plate)
PLATE_ORIENTATION_TOLERANCE = math.radians(5.0)
PLATE_ORIENTATION_STATUS_FILE = "/tmp/robot1_plate_orientation"

# Real wheel geometry, confirmed directly from physics.usda earlier this
# session -- wheel cylinder radius=0.04m, joint localPos y=+-0.087 (track
# width = 2*0.087 = 0.174m). Not guessed.
WHEEL_RADIUS = 0.04
TRACK_WIDTH = 0.174
WHEEL_JOINT_NAMES = ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"]

GRAPH_PATH = "/World/Ros2NavGraph"

def build_ros2_graph():
    """Odometry+TF publish, cmd_vel subscribe -> 4-wheel velocity drive.
    Every node type/attribute name here is confirmed directly from Isaac
    Sim's own installed .ogn definitions and its real
    test_differential_base.py test suite this session -- none guessed.

    DifferentialController always outputs exactly a 2-element
    [left, right] velocityCommand (confirmed via its .ogn: 'double[]'
    default [0.0, 0.0]) -- but this robot has 4 wheel joints, not 2. To
    avoid any uncertainty about ConstructArray's dynamic extra-input
    ports (only the arraySize=1/single-input0 case is confirmed by
    Isaac's own test suite), this uses FOUR separate
    IsaacArticulationController nodes, each with exactly one joint name
    and a matching one-element velocityCommand array -- front_left and
    rear_left share the same 'left' array node's output, front_right and
    rear_right share 'right', so all 4 wheels still track correctly.
    """
    keys = og.Controller.Keys
    graph, nodes, _, _ = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("GetLeft", "omni.graph.nodes.ArrayIndex"),
                ("GetRight", "omni.graph.nodes.ArrayIndex"),
                ("ConstructLeftCmd", "omni.graph.nodes.ConstructArray"),
                ("ConstructRightCmd", "omni.graph.nodes.ConstructArray"),
                ("ArtControllerFL", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ArtControllerRL", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ArtControllerFR", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ArtControllerRR", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishRawTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArtControllerFL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArtControllerRL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArtControllerFR.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArtControllerRR.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishScan.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishScan.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("DiffController.outputs:velocityCommand", "GetLeft.inputs:array"),
                ("DiffController.outputs:velocityCommand", "GetRight.inputs:array"),
                ("GetLeft.outputs:value", "ConstructLeftCmd.inputs:input0"),
                ("GetRight.outputs:value", "ConstructRightCmd.inputs:input0"),
                ("ConstructLeftCmd.outputs:array", "ArtControllerFL.inputs:velocityCommand"),
                ("ConstructLeftCmd.outputs:array", "ArtControllerRL.inputs:velocityCommand"),
                ("ConstructRightCmd.outputs:array", "ArtControllerFR.inputs:velocityCommand"),
                ("ConstructRightCmd.outputs:array", "ArtControllerRR.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("PublishOdom.inputs:topicName", "odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishRawTF.inputs:topicName", "tf"),
                ("PublishRawTF.inputs:parentFrameId", "odom"),
                ("PublishRawTF.inputs:childFrameId", "base_link"),
                ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", TRACK_WIDTH),
                ("GetLeft.inputs:index", 0),
                ("GetRight.inputs:index", 1),
                ("ConstructLeftCmd.inputs:arraySize", 1),
                ("ConstructRightCmd.inputs:arraySize", 1),
                ("ArtControllerFL.inputs:jointNames", ["front_left_wheel_joint"]),
                ("ArtControllerRL.inputs:jointNames", ["rear_left_wheel_joint"]),
                ("ArtControllerFR.inputs:jointNames", ["front_right_wheel_joint"]),
                ("ArtControllerRR.inputs:jointNames", ["rear_right_wheel_joint"]),
                ("ArtControllerFL.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("ArtControllerRL.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("ArtControllerFR.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("ArtControllerRR.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("PublishScan.inputs:frameId", "base_link"),
                ("PublishScan.inputs:topicName", "scan"),
                ("PublishScan.inputs:horizontalFov", 360.0),
                ("PublishScan.inputs:horizontalResolution", 360.0 / LASER_NUM_SAMPLES),
                ("PublishScan.inputs:depthRange", [0.02, SCAN_MAX_RANGE]),
                ("PublishScan.inputs:azimuthRange", [-180.0, 180.0]),
                ("PublishScan.inputs:numRows", 1),
                ("PublishScan.inputs:numCols", LASER_NUM_SAMPLES),
                ("PublishClock.inputs:topicName", "clock"),
            ],
        },
    )
    return graph, nodes

graph, nodes = build_ros2_graph()
print(f"[nav2 bridge robot1] built ROS2 OmniGraph at {GRAPH_PATH} with {len(nodes)} nodes")
for _ in range(5):
    simulation_app.update()

# Setup is done and physics tensors are initialized -- pause here so the
# robot sits still until you press Play yourself, instead of driving off
# during the rest of this script's startup.
omni.timeline.get_timeline_interface().pause()
print("[nav2 bridge robot1] paused -- press Play in the Isaac Sim window when ready")

# Cached once -- og.Controller.set() is called on this every tick with
# fresh camera data, entirely on the Kit/OmniGraph side (no rclpy).
_scan_data_attr = og.Controller.attribute(f"{GRAPH_PATH}/PublishScan.inputs:linearDepthData")

# Visual "go here" marker -- same design as robot1_cube03_live_test.py's
# spawn_waypoint_marker (real collision + RigidBodyAPI so it falls and
# actually lands/rests on the ground, but filtered via
# UsdPhysics.FilteredPairsAPI against every collision-bearing prim under
# Robot so it never physically touches or blocks it). Dropped from this
# script when it was first written -- restored so a goal position (or
# anywhere else) can be visually marked for live verification.
_marker_count = 0

def _robot_collision_prim_paths():
    return [
        prim.GetPath()
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH))
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]

def spawn_marker_at(x, y, label=""):
    """Drops a visual marker at a given WORLD (x, y) -- lands/rests on
    the ground, passes through Robot."""
    global _marker_count
    path = f"/Waypoint_{_marker_count}"
    _marker_count += 1
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.5))
    xf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    filt = UsdPhysics.FilteredPairsAPI.Apply(prim)
    filt.CreateFilteredPairsRel().SetTargets(_robot_collision_prim_paths())
    print(f"[nav2 bridge robot1] spawned marker {path} at world ({x:.3f},{y:.3f}) {label}")
    return path

def spawn_waypoint_marker(r, label):
    """O key -- drops a marker 0.5m ahead of r's current heading."""
    x, y, yaw = get_xy_yaw(r)
    ox = x + 0.5 * math.cos(yaw)
    oy = y + 0.5 * math.sin(yaw)
    spawn_marker_at(ox, oy, label=f"ahead of {label}")

# odom frame's origin is wherever the robot was when the bridge started,
# with its axes aligned to the robot's spawn heading (chassis_yaw0), not
# world axes -- confirmed directly this session: odom read ~0.009 at
# spawn, not the real world spawn coordinate ~2.04, and this robot spawns
# yawed 180 deg (confirmed in the stage's Orient Z), so an odom-frame
# offset must be rotated by chassis_yaw0 before adding to the spawn world
# position, not added directly.
_spawn_x, _spawn_y, _ = get_xy_yaw(robot)

def spawn_marker_at_odom_goal(odom_x, odom_y, label="goal"):
    c, s = math.cos(chassis_yaw0), math.sin(chassis_yaw0)
    wx = _spawn_x + odom_x * c - odom_y * s
    wy = _spawn_y + odom_x * s + odom_y * c
    return spawn_marker_at(wx, wy, label=label)

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])
        elif e.input == carb.input.KeyboardInput.O:
            spawn_waypoint_marker(robot, "Robot")
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)
print("[nav2 bridge robot1] wheels are driven by /cmd_vel (publish externally, e.g. via Nav2 or `ros2 topic pub`)")
print("[nav2 bridge robot1] R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame the scene, O drop a marker ahead of Robot")
print("[nav2 bridge robot1] publishing /odom + /tf, subscribing /cmd_vel")

Piston_speed = 0.15
Tilt_speed = 2
manual_yaw_target = yaw_joint_base
last_time = time.perf_counter()
_tick_count = 0
DEPTH_PRINT_INTERVAL = 60

# Remote play/pause control -- no GUI automation available to click the
# window's own Play button from outside the process, so this polls a
# plain control file each tick instead: `echo play >
# /tmp/robot1_play_control` / `echo pause > ...`. Doesn't reintroduce
# auto-play-at-launch (still starts paused, per this session's fix --
# nothing writes this file until explicitly told to).
PLAY_CONTROL_FILE = "/tmp/robot1_play_control"
_last_play_control = None
# Live wheel-damping sweep -- avoids a ~150s Isaac reboot per guess.
# `echo 5.0 > /tmp/robot1_damping`.
DAMPING_CONTROL_FILE = "/tmp/robot1_damping"
_last_damping = None
# Remote obstacle spawn, for testing replanning without a keyboard --
# `echo "-1.0,2.5" > /tmp/robot1_spawn_obstacle` drops a real (but
# robot-collision-filtered) marker at that ODOM-frame x,y, same object
# spawn_marker_at_odom_goal already uses for goals -- it's just as
# visible to /scan either way, which is all that matters for a costmap
# obstacle test.
SPAWN_OBSTACLE_FILE = "/tmp/robot1_spawn_obstacle"
_last_obstacle_spawn = None
# Remote position query -- lets an outside script read back where a prim
# (e.g. a marker the user manually dragged in the viewport) currently is,
# converted to odom frame same as goals/obstacles are specified in.
# `echo /Waypoint_0 > /tmp/robot1_query_marker`, then read
# /tmp/robot1_query_response for "odom_x,odom_y".
QUERY_MARKER_FILE = "/tmp/robot1_query_marker"
QUERY_RESPONSE_FILE = "/tmp/robot1_query_response"
# Real scene ground truth -- lists every direct child of /World with its
# type and REAL world-space bounding box (odom-converted), not just an
# Xform's pivot/origin. Added after repeatedly assuming a shelf/obstacle
# prim's Xform translate was a usable target point without ever checking
# its actual footprint.
SCENE_DUMP_FILE = "/tmp/robot1_scene_dump"
SCENE_DUMP_RESPONSE_FILE = "/tmp/robot1_scene_dump_response"
# Remote toggle for apply_yaw_compensation -- `echo off > .../robot1_yaw_compensation`
# suspends it (the plate just passively rides along with the chassis,
# no active counter-rotation), `echo on > ...` resumes it. Added so a
# long, multi-turn carry leg doesn't force the yaw joint to continuously
# chase a rapidly-changing target it isn't torqued for -- suspend during
# the noisy transit, only actively hold/correct orientation when it
# actually matters (stationary, at a precision waypoint). Defaults to
# on/enabled so every other existing use of this bridge is unaffected.
YAW_COMPENSATION_CONTROL_FILE = "/tmp/robot1_yaw_compensation"
_yaw_compensation_enabled = True
_last_yaw_compensation_cmd = None
# Remote pickup/place control -- `echo lift:/Cube > /tmp/robot1_piston_cmd`
# ramps to LIFT_TARGET and starts ground-truth cargo tracking on that
# prim; `echo lower > ...` ramps back to this robot's real rest position
# (PISTON_REST). Writes "lifted"/"lowered" to PISTON_STATUS_FILE once the
# ramp actually completes, so an external orchestrator can poll for real
# completion instead of guessing a sleep duration.
PISTON_CONTROL_FILE = "/tmp/robot1_piston_cmd"
PISTON_STATUS_FILE = "/tmp/robot1_piston_status"
_piston_remote_target = None  # None = no override, R/C keys still work
# Ground-truth obstacle-touch file, written every tick (not a polled
# command -- a live sensor value): "clear", or a comma-joined list of
# which bodies (chassis / the carried prim's path) currently overlap
# Cube_03's real footprint. See the AABB-overlap block below for why.
CARRY_OBSTACLE_FILE = "/tmp/robot1_carry_obstacle"
# Real world bbox for the shelves-world /Cube_03 obstacle, queried once via
# SCENE_DUMP_FILE this session -- not guessed, matches the "real position,
# queried directly... not guessed" discipline robot1_cube03_live_test.py
# uses for its own CUBE03_X/CUBE03_Y.
CUBE03_WORLD_BBOX = ((0.225, 1.750), (1.225, 2.750))  # (min_x,min_y),(max_x,max_y)
# Updated 2026-08-15: Cube_03 moved from world Y-center 1.328 to 2.25
# (shelf-2/obstacle relayout, obstacle now at "+3 from shelf 1" on its own
# clean axis instead of sitting close enough to shelf 1's own structure to
# be ambiguous with it) -- re-queried via SCENE_DUMP_FILE's world_bbox=
# field after the move, not hand-computed.
CHASSIS_HALF_X, CHASSIS_HALF_Y = 0.15, 0.075  # same real chassis half-extents occupancy_grid.py uses
_touch_bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
_obstacle_touching = False

def _aabb_overlap(cx, cy, half_x, half_y, bbox):
    (bx0, by0), (bx1, by1) = bbox
    return (cx + half_x >= bx0 and cx - half_x <= bx1 and
            cy + half_y >= by0 and cy - half_y <= by1)
# Remote clearance diagnostic -- `echo x > /tmp/robot1_diag` writes real
# wheel velocities (target vs actual) and raycasts from the chassis and
# (if carrying something) the cargo, in the 4 world-axis directions, to
# /tmp/robot1_diag_response. Ground truth, not reasoning about mount
# geometry on paper -- same discipline as two_robot_pickup_demo.py's
# identify_obstacle() from earlier sessions.
DIAG_FILE = "/tmp/robot1_diag"
DIAG_RESPONSE_FILE = "/tmp/robot1_diag_response"

while simulation_app.is_running():
    now = time.perf_counter()
    dt = now - last_time
    last_time = now

    if os.path.exists(PLAY_CONTROL_FILE):
        with open(PLAY_CONTROL_FILE) as f:
            _cmd = f.read().strip()
        if _cmd != _last_play_control:
            if _cmd == "play":
                omni.timeline.get_timeline_interface().play()
                print("[nav2 bridge robot1] play (remote)", flush=True)
            elif _cmd == "pause":
                omni.timeline.get_timeline_interface().pause()
                print("[nav2 bridge robot1] pause (remote)", flush=True)
            _last_play_control = _cmd

    if os.path.exists(YAW_COMPENSATION_CONTROL_FILE):
        with open(YAW_COMPENSATION_CONTROL_FILE) as f:
            _yc_cmd = f.read().strip()
        if _yc_cmd != _last_yaw_compensation_cmd:
            if _yc_cmd == "off":
                _yaw_compensation_enabled = False
                print("[nav2 bridge robot1] yaw compensation SUSPENDED (remote)", flush=True)
            elif _yc_cmd == "on":
                _yaw_compensation_enabled = True
                print("[nav2 bridge robot1] yaw compensation RESUMED (remote)", flush=True)
            _last_yaw_compensation_cmd = _yc_cmd

    if os.path.exists(DAMPING_CONTROL_FILE):
        with open(DAMPING_CONTROL_FILE) as f:
            _damping_cmd = f.read().strip()
        if _damping_cmd != _last_damping:
            try:
                robot.set_dof_gains(stiffnesses=0, dampings=float(_damping_cmd), dof_indices=wheel_joints)
                print(f"[nav2 bridge robot1] wheel damping -> {_damping_cmd} (remote)", flush=True)
            except ValueError:
                pass
            _last_damping = _damping_cmd

    if os.path.exists(SPAWN_OBSTACLE_FILE):
        with open(SPAWN_OBSTACLE_FILE) as f:
            _obs_cmd = f.read().strip()
        if _obs_cmd != _last_obstacle_spawn:
            try:
                ox, oy = (float(v) for v in _obs_cmd.split(","))
                spawn_marker_at_odom_goal(ox, oy, label="obstacle (remote)")
            except ValueError:
                pass
            _last_obstacle_spawn = _obs_cmd

    if os.path.exists(QUERY_MARKER_FILE):
        with open(QUERY_MARKER_FILE) as f:
            _query_path = f.read().strip()
        # Trigger on the response file being absent (the client's own
        # signal that it wants a fresh answer -- it deletes the response
        # before writing the marker), not on the marker content changing --
        # a content-equality check silently drops a repeat query for the
        # same prim, which a real caller does all the time.
        if _query_path and not os.path.exists(QUERY_RESPONSE_FILE):
            prim = stage.GetPrimAtPath(_query_path)
            if prim.IsValid():
                wt = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                wx, wy, wz = wt.ExtractTranslation()
                c, s = math.cos(-chassis_yaw0), math.sin(-chassis_yaw0)
                dx, dy = wx - _spawn_x, wy - _spawn_y
                odom_x = dx * c - dy * s
                odom_y = dx * s + dy * c
                # wz needs no odom transform -- the world<->odom rotation is
                # about the Z axis only (spawn yaw), so Z passes through
                # unchanged. Appended as a 3rd field -- existing callers
                # that only unpack 2 values (query_prim_odom_xy) are
                # untouched, this is purely additive.
                with open(QUERY_RESPONSE_FILE, "w") as out:
                    out.write(f"{odom_x:.4f},{odom_y:.4f},{wz:.4f}")

    if os.path.exists(SCENE_DUMP_FILE):
        with open(SCENE_DUMP_FILE) as f:
            _dump_cmd = f.read().strip()
        if _dump_cmd and not os.path.exists(SCENE_DUMP_RESPONSE_FILE):
            _bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            _c, _s = math.cos(-chassis_yaw0), math.sin(-chassis_yaw0)
            def _to_odom(wx, wy):
                _dx, _dy = wx - _spawn_x, wy - _spawn_y
                return _dx * _c - _dy * _s, _dx * _s + _dy * _c
            _lines = []
            for _child in stage.GetPrimAtPath("/World").GetChildren():
                try:
                    _rng = _bbox_cache.ComputeWorldBound(_child).ComputeAlignedRange()
                    _mn, _mx = _rng.GetMin(), _rng.GetMax()
                    _ox0, _oy0 = _to_odom(_mn[0], _mn[1])
                    _ox1, _oy1 = _to_odom(_mx[0], _mx[1])
                    _lines.append(f"{_child.GetPath()} [{_child.GetTypeName()}] "
                                  f"world_bbox=({_mn[0]:.3f},{_mn[1]:.3f},{_mn[2]:.3f})-({_mx[0]:.3f},{_mx[1]:.3f},{_mx[2]:.3f}) "
                                  f"odom_bbox=({_ox0:.3f},{_oy0:.3f})-({_ox1:.3f},{_oy1:.3f})")
                except Exception as _e:
                    _lines.append(f"{_child.GetPath()} [{_child.GetTypeName()}] bbox error: {_e}")
            # A direct-children-only dump misses anything nested under a
            # group (e.g. a prop parented under a shelf's Xform instead of
            # sitting directly under /World) -- full recursive scan for
            # actual geometry types, skipping the robots' own internal
            # links (pure noise -- dozens of sub-links per robot). Scoped
            # to the WHOLE STAGE, not just /World -- confirmed live this
            # session that /Cube (the actual carried payload) is a
            # root-level sibling of /World, not nested under it, so a
            # /World-only scan structurally cannot see it or anything
            # else living at that same root level (e.g. a real obstacle).
            _lines.append("--- full recursive geometry scan (whole stage, excluding /World/Robot*) ---")
            for _prim in Usd.PrimRange(stage.GetPseudoRoot()):
                _path = str(_prim.GetPath())
                if _path.startswith("/World/Robot"):
                    continue
                if _prim.GetTypeName() not in ("Mesh", "Cube", "Sphere", "Cylinder", "Cone"):
                    continue
                try:
                    _rng = _bbox_cache.ComputeWorldBound(_prim).ComputeAlignedRange()
                    _mn, _mx = _rng.GetMin(), _rng.GetMax()
                    _ox0, _oy0 = _to_odom(_mn[0], _mn[1])
                    _ox1, _oy1 = _to_odom(_mx[0], _mx[1])
                    _has_collision = _prim.HasAPI(UsdPhysics.CollisionAPI)
                    _lines.append(f"{_path} [{_prim.GetTypeName()}] collision={_has_collision} "
                                  f"world_bbox=({_mn[0]:.3f},{_mn[1]:.3f},{_mn[2]:.3f})-({_mx[0]:.3f},{_mx[1]:.3f},{_mx[2]:.3f}) "
                                  f"odom_bbox=({_ox0:.3f},{_oy0:.3f})-({_ox1:.3f},{_oy1:.3f})")
                except Exception as _e:
                    _lines.append(f"{_path} [{_prim.GetTypeName()}] bbox error: {_e}")
            with open(SCENE_DUMP_RESPONSE_FILE, "w") as out:
                out.write("\n".join(_lines))

    if os.path.exists(PISTON_CONTROL_FILE):
        with open(PISTON_CONTROL_FILE) as f:
            _piston_cmd = f.read().strip()
        # Consume-and-delete, not a content-equality gate -- a leftover
        # command file from a crashed prior run (confirmed live: a stale
        # "lift:/Cube" survived a bridge restart and replayed itself into
        # the piston target on the very first tick of the fresh process,
        # lifting before Play was even pressed and before anything asked
        # for it) must never be able to fire again just because a new
        # process's _last_piston_cmd starts at None.
        os.remove(PISTON_CONTROL_FILE)
        if _piston_cmd:
            if _piston_cmd.startswith("lift"):
                parts = _piston_cmd.split(":", 1)
                carrying_path = parts[1] if len(parts) > 1 else None
                _piston_remote_target = LIFT_TARGET
                print(f"[nav2 bridge robot1] piston lift -> {carrying_path} (remote)", flush=True)
            elif _piston_cmd == "lower":
                _piston_remote_target = PISTON_REST
                print("[nav2 bridge robot1] piston lower (remote)", flush=True)

    if _piston_remote_target is not None:
        if piston_target < _piston_remote_target:
            piston_target = min(_piston_remote_target, piston_target + Piston_speed * dt)
        else:
            piston_target = max(_piston_remote_target, piston_target - Piston_speed * dt)
        robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)
        if abs(piston_target - _piston_remote_target) < 1e-4:
            reached_lift = _piston_remote_target == LIFT_TARGET
            with open(PISTON_STATUS_FILE, "w") as out:
                out.write("lifted" if reached_lift else "lowered")
            if reached_lift and carrying_path:
                ox, oy, oz = RigidPrim(carrying_path).get_world_poses()[0].numpy()[0]
                cx, cy, _ = get_xy_yaw(robot)
                carrying_offset = (float(ox - cx), float(oy - cy), float(oz))
                with open(CARGO_STATUS_FILE, "w") as out:
                    out.write(f"attached:{carrying_path}")
            else:
                carrying_path = None
                carrying_offset = None
                with open(CARGO_STATUS_FILE, "w") as out:
                    out.write("none")
            _piston_remote_target = None

    if carrying_path and carrying_offset is not None:
        ox, oy, oz = RigidPrim(carrying_path).get_world_poses()[0].numpy()[0]
        cx, cy, _ = get_xy_yaw(robot)
        exp_dx, exp_dy, exp_z = carrying_offset
        horiz_drift = math.hypot((ox - cx) - exp_dx, (oy - cy) - exp_dy)
        if oz < exp_z - CARGO_DROP_TOLERANCE or horiz_drift > CARGO_DROP_TOLERANCE:
            print(f"[nav2 bridge robot1] WARNING: {carrying_path} came off the plate -- "
                  f"z {exp_z:.3f} -> {oz:.3f}, horizontal drift {horiz_drift:.3f}m", flush=True)
            carrying_path = None
            carrying_offset = None
            with open(CARGO_STATUS_FILE, "w") as out:
                out.write("dropped")

    # Ground-truth "did we actually touch the obstacle" test trigger. The
    # raycast_from_excluding() approach this block used to run was a dead
    # end: /Cube_03 (the real obstacle, confirmed via SCENE_DUMP_FILE --
    # world_bbox=(0.225,0.828,0.074)-(1.225,1.828,1.074), collision=False)
    # has no physics collision, so a PhysX raycast can never see it,
    # matching the earlier single-robot+obstacle test's own collision-less
    # prop. Per direct instruction, this checks real, known 2D AABB overlap
    # instead -- chassis position (+/- its real half-extents, same
    # CHASSIS_HALF_X/Y occupancy_grid.py uses) and, if carrying something,
    # that object's own live bbox (BBoxCache, not a guessed half-extent)
    # against Cube_03's real bbox queried above. This is independent of
    # whatever avoidance logic is running -- it's ground truth for whether
    # contact actually happened, used to verify the fix, not a detection
    # mechanism the controller reacts to.
    _cx, _cy, _cyaw = get_xy_yaw(robot)
    _touch_bodies = []
    if _aabb_overlap(_cx, _cy, CHASSIS_HALF_X, CHASSIS_HALF_Y, CUBE03_WORLD_BBOX):
        _touch_bodies.append("chassis")
    if carrying_path:
        try:
            _cargo_rng = _touch_bbox_cache.ComputeWorldBound(
                stage.GetPrimAtPath(carrying_path)).ComputeAlignedRange()
            _cmn, _cmx = _cargo_rng.GetMin(), _cargo_rng.GetMax()
            if _cmx[0] >= CUBE03_WORLD_BBOX[0][0] and _cmn[0] <= CUBE03_WORLD_BBOX[1][0] and \
               _cmx[1] >= CUBE03_WORLD_BBOX[0][1] and _cmn[1] <= CUBE03_WORLD_BBOX[1][1]:
                _touch_bodies.append(carrying_path)
        except Exception:
            pass
    _now_touching = bool(_touch_bodies)
    with open(CARRY_OBSTACLE_FILE, "w") as out:
        out.write(",".join(_touch_bodies) if _touch_bodies else "clear")
    if _now_touching and not _obstacle_touching:
        print(f"[nav2 bridge robot1] *** OBSTACLE TOUCH TRIGGERED *** {_touch_bodies} overlapping "
              f"Cube_03's real footprint -- chassis=({_cx:.3f},{_cy:.3f}) carrying={carrying_path}", flush=True)
    elif _obstacle_touching and not _now_touching:
        print("[nav2 bridge robot1] obstacle touch cleared", flush=True)
    _obstacle_touching = _now_touching

    _, _, plate_yaw_now = get_xy_yaw(plate)
    plate_yaw_err = math.atan2(math.sin(plate_yaw_now - plate_yaw0), math.cos(plate_yaw_now - plate_yaw0))
    if abs(plate_yaw_err) > PLATE_ORIENTATION_TOLERANCE:
        print(f"[nav2 bridge robot1] WARNING: plate orientation drifted {math.degrees(plate_yaw_err):.1f}deg from startup", flush=True)
        with open(PLATE_ORIENTATION_STATUS_FILE, "w") as out:
            out.write(f"drifted:{math.degrees(plate_yaw_err):.1f}")
    else:
        with open(PLATE_ORIENTATION_STATUS_FILE, "w") as out:
            out.write("ok")

    if os.path.exists(DIAG_FILE):
        with open(DIAG_FILE) as f:
            _diag_cmd = f.read().strip()
        # Trigger on the response file being absent, not on the request
        # content changing -- a content-equality gate silently drops a
        # repeat request for the same command, exactly like the query-file
        # bug found and fixed earlier this session.
        if _diag_cmd and not os.path.exists(DIAG_RESPONSE_FILE):
            lines = []
            wvel = robot.get_dof_velocities(dof_indices=wheel_joints).numpy()[0]
            lines.append(f"wheel velocities (actual): {wvel.tolist()}")
            lines.append(f"max_abs_wheel_velocity: {float(max(abs(v) for v in wvel)):.4f}")
            cx, cy, cyaw = get_xy_yaw(robot)
            lines.append(f"chassis pos=({cx:.3f},{cy:.3f}) yaw={math.degrees(cyaw):.1f}deg")
            for label, (dx, dy) in [("+X", (1, 0)), ("-X", (-1, 0)), ("+Y", (0, 1)), ("-Y", (0, -1))]:
                body, dist = raycast_from((cx, cy, 0.15), (dx, dy))
                lines.append(f"  chassis raycast {label}: {body} @ {dist}")
            if carrying_path:
                ox, oy, oz = RigidPrim(carrying_path).get_world_poses()[0].numpy()[0]
                lines.append(f"cargo {carrying_path} pos=({ox:.3f},{oy:.3f},{oz:.3f})")
                for label, (dx, dy) in [("+X", (1, 0)), ("-X", (-1, 0)), ("+Y", (0, 1)), ("-Y", (0, -1))]:
                    body, dist = raycast_from((ox, oy, oz), (dx, dy))
                    lines.append(f"  cargo raycast {label}: {body} @ {dist}")
            with open(DIAG_RESPONSE_FILE, "w") as out:
                out.write("\n".join(lines))

    if _yaw_compensation_enabled:
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)

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

    if yaw_left or yaw_right:
        # Manual keyboard override -- only write to the yaw joint while a
        # key is actually held. This used to run unconditionally every
        # tick regardless of key state, which silently overwrote
        # apply_yaw_compensation's counter-rotation target immediately
        # after it was set a few lines above (same joint, called later in
        # the same tick) -- the counter-rotation was computing correctly
        # but never actually reaching the joint. Confirmed live this
        # session: the carried object was swinging with the chassis
        # during a turn instead of holding its orientation, and this
        # leftover manual-control code (predating the Nav2 migration,
        # never updated for it) was the real cause, not a speed/lag issue.
        if yaw_left:
            manual_yaw_target += Tilt_speed * dt
        elif yaw_right:
            manual_yaw_target -= Tilt_speed * dt
        robot.set_dof_position_targets(manual_yaw_target, dof_indices=yaw_joint)
    else:
        # get_dof_positions has crashed the whole process twice tonight in
        # GUI mode (both idle-paused and during active play) with "physics
        # tensor entity is not valid" -- a real Isaac Sim quirk, not
        # something fixable from here. Skipping one tick's resync on a
        # transient failure is harmless (manual_yaw_target only matters
        # when Q/E is actually pressed) and far better than losing the
        # whole process and every other check running this tick.
        try:
            manual_yaw_target = float(robot.get_dof_positions(dof_indices=yaw_joint).numpy()[0, 0])
        except AssertionError as _e:
            print(f"[nav2 bridge robot1] WARNING: get_dof_positions failed this tick ({_e}) -- skipping manual_yaw_target resync", flush=True)

    og.Controller.set(_scan_data_attr, synthesize_laser_ranges(depth_cams1))

    simulation_app.update()
    _tick_count += 1
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        x, y, yaw = get_xy_yaw(robot)
        print(f"[nav2 bridge robot1] pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f}")
