"""Sensor calibration run: validates the live nav stack's raycast-based
ranging (depth_cameras.range_at_body_angle) against an INDEPENDENT
ground-truth raycast, in isaac/test/camera_calibration.usd (built by
test/build_camera_calibration_world.py).

Ground truth uses a DIFFERENT way of clearing the robot's own piston
housing than production does (raycasting from directly above it, at
chassis-center X/Y, instead of production's small same-direction XY
offset) -- deliberately independent so this is a real cross-check, not a
circular comparison of a raycast against itself.

This script's history is worth knowing before reading it: it originally
compared each of 4 CORNER-mounted cameras' RENDERED depth images against
raycast ground truth. That surfaced two real, serious bugs in turn: (1)
a parallax miss on close objects (fixed by moving to 5 center-mounted
cameras), then (2) a deeper problem where the cameras' own rendered
depth images turned out to not match real geometry once more than one
object shared a camera's field of view -- reproduced under both the
low-VRAM MinimalRendering path and the full renderer, so it wasn't a
renderer-quality issue, and at multiple mount distances, so it wasn't a
proximity issue either. Every PhysX raycast run this session, by
contrast, was accurate every time. Given that, the live nav stack's
ranging itself was switched from rendered-camera-pixel sampling to a
direct raycast fan (see depth_cameras.py's module docstring for the
full account) -- so THIS script's job changed from "characterize the
camera" to "confirm the raycast-based ranging that replaced it is
correct."

Results go to logs/camera_calibration_results.csv.

Static scene -- the robot never moves here, this is purely a sensing
characterization run, not a navigation test.

Usage:
    PYTHONPATH= AMENT_PREFIX_PATH= COLCON_PREFIX_PATH= HEADLESS=1 \\
        ./python.sh camera_calibration.py
"""
import csv
import json
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": os.environ.get("HEADLESS", "1") == "1",
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
})

import omni.timeline
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from omni.physx import get_physx_scene_query_interface
from pxr import PhysxSchema

import debug_viz
import depth_cameras

ISAAC_DIR = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(ISAAC_DIR, "test", "camera_calibration.usd")
REFERENCE_JSON_PATH = os.path.join(ISAAC_DIR, "test", "camera_calibration_reference.json")
RESULTS_CSV_PATH = os.path.join(os.path.dirname(ISAAC_DIR), "logs", "camera_calibration_results.csv")
ROBOT_PATH = "/World/Robot/Geometry/base_link"
# Ground-truth raycast height -- clear of actuator_outer_cylinder_link
# (real world bbox z=[0.115,0.153]) via HEIGHT alone, from exact chassis
# center X/Y, deliberately different from production's small-XY-offset
# approach at CAMERA_HEIGHT so this is an independent cross-check.
GROUND_TRUTH_HEIGHT = 0.20

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

success, stage = stage_utils.open_stage(USD_PATH)
print(f"[camera_calibration] opened {USD_PATH} -> {success}")
for _ in range(5):
    simulation_app.update()

debug_viz.clear_debug_vis(stage)
debug_viz.init_debug_vis(stage)

art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROBOT_PATH))
art_api.CreateSolverPositionIterationCountAttr(255)
art_api.CreateSolverVelocityIterationCountAttr(64)

# Physics tensor API needs the timeline played at least once (same
# constraint as nav2_bridge_robot1.py), before any DOF read/write.
omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

# Live-verified this session: leaving the wheel joints entirely
# unconfigured (no damping, no explicit zero velocity target) let them
# spin freely and continuously under whatever leftover drive state the
# source USD carried -- confirmed via direct DOF position reads. Mirrors
# nav2_bridge_robot1.py's setup_robot() so the robot is genuinely static
# for this calibration run, not just visually still.
WHEEL_DOF_DAMPING = 5000.0
robot_art = Articulation(ROBOT_PATH)
wheel_dofs = robot_art.get_dof_indices(
    ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
robot_art.set_dof_gains(stiffnesses=0, dampings=WHEEL_DOF_DAMPING, dof_indices=wheel_dofs)
robot_art.set_dof_velocity_targets(0, dof_indices=wheel_dofs)

for _ in range(25):
    simulation_app.update()

with open(REFERENCE_JSON_PATH) as f:
    reference = json.load(f)
print(f"[camera_calibration] loaded {len(reference['objects'])} reference objects")

_pos, _quat = robot_art.get_world_poses()
_p = _pos.numpy()[0]
_w, _x, _y, _z = _quat.numpy()[0]
CHASSIS_X, CHASSIS_Y = float(_p[0]), float(_p[1])
CHASSIS_YAW = math.atan2(2 * (_w * _z + _x * _y), 1 - 2 * (_y * _y + _z * _z))
print(f"[camera_calibration] chassis world pose: ({CHASSIS_X:.4f},{CHASSIS_Y:.4f}) yaw={math.degrees(CHASSIS_YAW):.2f}deg")

_physx_query = get_physx_scene_query_interface()


def ground_truth_range(bearing_rad, max_dist=10.0):
    """Independent ground-truth raycast: chassis-center X/Y, cleared of
    the piston housing via HEIGHT alone (not production's small XY
    offset) -- see module docstring for why this stays a real
    cross-check instead of comparing a raycast against itself."""
    world_bearing = CHASSIS_YAW + bearing_rad
    direction = (math.cos(world_bearing), math.sin(world_bearing), 0.0)
    origin = (CHASSIS_X, CHASSIS_Y, GROUND_TRUTH_HEIGHT)
    hit = _physx_query.raycast_closest(origin, direction, max_dist)
    return float(hit["distance"]) if hit["hit"] else None


# Camera prims are still created (FOV-cone visualization only -- no
# longer part of the active ranging path, see depth_cameras.py).
depth_cameras.add_camera_rig(ROBOT_PATH, simulation_app, stage=stage)

rows = []
for obj in reference["objects"]:
    bearing_deg = obj["bearing_deg_from_robot_forward"]
    bearing_rad = math.radians(bearing_deg)
    design_dist = obj["true_distance_m"]  # build-time center-to-center distance -- informational only

    prod_range = depth_cameras.range_at_body_angle(CHASSIS_X, CHASSIS_Y, CHASSIS_YAW, bearing_rad)
    gt_range = ground_truth_range(bearing_rad)

    bias = (prod_range - gt_range) if (prod_range is not None and gt_range is not None) else None
    row = {
        "label": obj["label"], "kind": obj["kind"],
        "bearing_deg": bearing_deg,
        "design_distance_m": design_dist,
        "ground_truth_m": gt_range, "sensed_range_m": prod_range, "bias_m": bias,
    }
    rows.append(row)
    gt_str = "n/a" if gt_range is None else f"{gt_range:.3f}m"
    sensed_str = "n/a" if prod_range is None else f"{prod_range:.3f}m"
    sensed_str += "" if bias is None else f" (bias {bias:+.4f}m)"
    print(f"[camera_calibration] {obj['label']:>12s} design={design_dist:.3f}m "
          f"ground_truth={gt_str} sensed={sensed_str}")

os.makedirs(os.path.dirname(RESULTS_CSV_PATH), exist_ok=True)
with open(RESULTS_CSV_PATH, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)
print(f"[camera_calibration] wrote {RESULTS_CSV_PATH}")

# In GUI mode, stay open so the scene/FOV cones can actually be
# inspected -- only auto-close in headless runs (batch/CI use).
if os.environ.get("HEADLESS", "1") == "1":
    simulation_app.close()
else:
    from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
    frame_viewport_prims(get_active_viewport(), [ROBOT_PATH, "/World/RefObjects"])
    print("[camera_calibration] staying open for inspection -- close the window when done")
    while simulation_app.is_running():
        simulation_app.update()
