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
from isaacsim.sensors.camera import Camera
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, Gf

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

# Enable the ROS2 bridge BEFORE the graph is built -- matches every real
# Isaac Sim ROS2 example (subscriber.py, carter_multiple_robot_navigation.py).
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "robot1_cube03_test.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[nav2 bridge robot1] opened {USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])

art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/Robot/Geometry/base_link"))
art_api.CreateSolverPositionIterationCountAttr(255)
art_api.CreateSolverVelocityIterationCountAttr(64)

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

ROBOT_PATH = "/World/Robot/Geometry/base_link"

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

robot, wheel_joints, tilt_joints, yaw_joint, yaw_joint_base, piston_joint, piston_target = setup_robot(ROBOT_PATH)

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

def synthesize_laser_ranges(cams):
    """One range per fixed robot-frame angle (LASER_NUM_SAMPLES evenly
    spaced from -pi to pi, matching ROS2PublishLaserScan's azimuthRange
    [-180,180]), picking whichever corner camera's FOV covers that
    direction. Returned in increasing-azimuth order, as the node
    requires."""
    ranges = []
    for i in range(LASER_NUM_SAMPLES):
        angle = LASER_ANGLE_MIN + i * LASER_ANGLE_INC
        best = SCAN_MAX_RANGE
        for cam, (_, _, cam_yaw_deg) in zip(cams, CAMERA_CORNERS):
            local_angle = angle - math.radians(cam_yaw_deg)
            local_angle = math.atan2(math.sin(local_angle), math.cos(local_angle))
            r = _sample_camera_range(cam, local_angle)
            if r is not None:
                best = min(best, r)
        ranges.append(best)
    return ranges

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

_, _, chassis_yaw0 = get_xy_yaw(robot)

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

# Mark the same goal sent in the earlier successful test run, so it's
# visible immediately without needing a keypress.
spawn_marker_at_odom_goal(-2.532, 3.997, label="nav2 goal")

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

while simulation_app.is_running():
    now = time.perf_counter()
    dt = now - last_time
    last_time = now

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

    if yaw_left:
        manual_yaw_target += Tilt_speed * dt
    elif yaw_right:
        manual_yaw_target -= Tilt_speed * dt
    robot.set_dof_position_targets(manual_yaw_target, dof_indices=yaw_joint)

    og.Controller.set(_scan_data_attr, synthesize_laser_ranges(depth_cams1))

    simulation_app.update()
    _tick_count += 1
    if _tick_count % DEPTH_PRINT_INTERVAL == 0:
        x, y, yaw = get_xy_yaw(robot)
        print(f"[nav2 bridge robot1] pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f}")
