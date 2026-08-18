"""Robot 1's ranging "sensor rig" and LaserScan synthesis, shared between
the live nav stack (nav2_bridge_robot1.py) and the camera-calibration
script (camera_calibration.py). Extracted so calibration measures the
EXACT setup actually used in production, not a re-implemented
approximation -- both callers get identical mounting and ranging logic
from one place.

History, in order, all from live testing this session (not guessed):
1. Started as 4 depth CAMERAS mounted at the corners (~0.15-0.17m out),
   using each camera's own rendered depth image, sampled per pixel
   column, to build a synthesized LaserScan.
2. Found a real parallax bug: a query for "the robot's" bearing X was
   answered by whichever corner camera covered X, but that camera sat
   far from chassis center -- fine for distant objects, but for a close
   (1.0m) target this could point the camera at empty space next to it
   (confirmed by tracing the exact geometry: an ~8deg miss). Redesigned
   to 5 cameras mounted close to a shared center point instead of 4 at
   the corners, which fixes this at the root for any distance.
3. That surfaced a SECOND, deeper problem: even mounted correctly, the
   cameras' own RENDERED depth images turned out to be unreliable once
   more than one real object shared a camera's wide (90deg) field of
   view -- a live raycast-vs-camera comparison showed the real geometry
   has sharp distance jumps between separate objects (confirmed via
   PhysX raycast, which was accurate in every single test run this
   session), while the camera's rendered depth reported one smooth,
   nearly-constant value across the ENTIRE frame regardless of what was
   actually where. Reproduced identically under both the low-VRAM
   MinimalRendering path this project normally uses AND the full
   renderer, and at multiple mount distances -- ruling out both as the
   cause. This looks like a real limitation of Isaac Sim's own
   depth-camera rendering for this kind of scene, not something fixable
   from calling code.
4. Given every PhysX raycast this session was accurate, every time,
   switched the actual ranging mechanism from "rendered camera pixels"
   to a direct PhysX raycast fan -- effectively a virtual multi-beam
   LiDAR standing in for what the camera rig conceptually represents.
   The 5 camera prims are still created (useful for FOV-cone
   visualization and possible future RGB use) but are no longer in the
   active ranging path.
"""
import math

import numpy as np
from isaacsim.sensors.camera import Camera
from omni.physx import get_physx_scene_query_interface

import debug_viz

CAMERA_HEIGHT = 0.10
CAMERA_MOUNT_RADIUS = 0.10  # small, equidistant offset from chassis
                             # center in each ray's OWN direction -- not
                             # zero, because the chassis center at this
                             # height is physically inside
                             # actuator_outer_cylinder_link (the piston
                             # housing, real world bbox z=[0.115,0.153]),
                             # confirmed via direct PhysX raycast earlier
                             # this session (a center-mounted sensor
                             # self-hit at distance 0.000m). Offsetting
                             # the raycast ORIGIN along the exact ray
                             # direction (not a fixed camera bearing)
                             # clears the housing in every direction with
                             # zero added parallax error, since the
                             # offset is colinear with the ray itself.
CAMERA_BEARINGS_DEG = [0.0, 72.0, 144.0, -144.0, -72.0]  # 360/5, one
                             # camera dead-center on "forward" (bearing
                             # 0) instead of straddling it between two
                             # cameras' edges. Only used for the visual
                             # camera rig now (FOV cones); ranging uses a
                             # dedicated raycast per queried angle, not
                             # these 5 fixed bearings.
CAMERA_MOUNTS = [
    (f"C{i}", np.array([
        CAMERA_MOUNT_RADIUS * math.cos(math.radians(bearing)),
        CAMERA_MOUNT_RADIUS * math.sin(math.radians(bearing)),
        CAMERA_HEIGHT,
    ]), bearing)
    for i, bearing in enumerate(CAMERA_BEARINGS_DEG)
]

SCAN_HORIZONTAL_FOV_DEG = 90.0  # matches set_focal_length(10.5) below --
                                  # cosmetic now (FOV cone visualization
                                  # only), since ranging no longer reads
                                  # camera pixels.
SCAN_MAX_RANGE = 3.0
LASER_NUM_SAMPLES = 128
LASER_ANGLE_MIN = -math.pi
LASER_ANGLE_MAX = math.pi
LASER_ANGLE_INC = (LASER_ANGLE_MAX - LASER_ANGLE_MIN) / LASER_NUM_SAMPLES

_physx_query = get_physx_scene_query_interface()


def _yaw_quat(deg):
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def add_depth_camera(base_link_path, name, translation, orientation, simulation_app):
    cam = Camera(
        prim_path=f"{base_link_path}/{name}",
        translation=translation,
        orientation=orientation,
        frequency=20,
        resolution=(128, 128),
    )
    cam.initialize()
    cam.set_clipping_range(near_distance=0.02, far_distance=50.0)
    cam.set_focal_length(10.5)  # ~90deg FOV
    cam.add_distance_to_camera_to_frame()
    for _ in range(30):
        simulation_app.update()
    return cam


def verify_no_self_hit(base_link_path, max_check_dist=0.3, num_directions=24):
    """Real PhysX raycast swept around a full circle at CAMERA_MOUNT_RADIUS,
    checking the offset genuinely clears the robot's own body in every
    direction (not just the 5 camera bearings) -- the failure mode that
    caused a real self-occlusion bug on a center-mounted sensor earlier
    this session. Best-effort, never raises."""
    try:
        query = _physx_query
    except Exception as e:
        print(f"[depth_cameras] WARNING: verify_no_self_hit unavailable ({e}) -- skipping")
        return []
    results = []
    for i in range(num_directions):
        bearing = 360.0 * i / num_directions
        try:
            direction = (math.cos(math.radians(bearing)), math.sin(math.radians(bearing)), 0.0)
            origin = (
                CAMERA_MOUNT_RADIUS * math.cos(math.radians(bearing)),
                CAMERA_MOUNT_RADIUS * math.sin(math.radians(bearing)),
                CAMERA_HEIGHT,
            )
            hit = query.raycast_closest(origin, direction, max_check_dist)
            if hit["hit"]:
                results.append((bearing, False, float(hit["distance"])))
                print(f"[depth_cameras] WARNING: self-hit at bearing {bearing:.0f}deg, "
                      f"{hit['distance']:.3f}m (body={hit.get('rigidBody', '?')})")
            else:
                results.append((bearing, True, None))
        except Exception as e:
            print(f"[depth_cameras] WARNING: verify_no_self_hit failed at bearing {bearing:.0f}deg ({e}) -- skipping")
    return results


def add_camera_rig(base_link_path, simulation_app, stage=None):
    """Creates the 5 star-mounted camera prims (FOV-cone visualization
    and possible future RGB use -- NOT the active ranging path, see
    module docstring). If `stage` is given, also draws a best-effort FOV
    cone per camera via debug_viz (never raises -- a cone failure just
    logs and continues)."""
    cams = [
        add_depth_camera(base_link_path, f"DepthCamera_{name}", pos, _yaw_quat(bearing), simulation_app)
        for name, pos, bearing in CAMERA_MOUNTS
    ]
    verify_no_self_hit(base_link_path)
    if stage is not None:
        for name, pos, bearing in CAMERA_MOUNTS:
            try:
                debug_viz.create_camera_fov_cone(
                    stage, base_link_path, f"FOVCone_{name}", pos, bearing,
                    fov_deg=SCAN_HORIZONTAL_FOV_DEG, range_m=SCAN_MAX_RANGE)
            except Exception as e:
                print(f"[depth_cameras] WARNING: FOV cone for DepthCamera_{name} failed ({e}) -- continuing")
    return cams


SELF_HIT_RETRY_EPSILON = 0.01  # meters -- how far past a self-hit to
                             # resume the raycast from, when skipping it
ROBOT_PATH_PREFIX = "/World/Robot/"  # any hit whose rigidBody starts
                             # with this is the robot's own body, never
                             # a real obstacle -- see range_at_world_pose.


def range_at_world_pose(chassis_x, chassis_y, chassis_yaw, body_angle_rad, max_range=SCAN_MAX_RANGE):
    """The real, PhysX-raycast-measured range at one body-frame angle
    (radians, 0 = chassis forward, increasing counterclockwise), from the
    robot's live world pose. This is the active ranging mechanism (see
    module docstring for why it replaced rendered-camera-pixel sampling).

    Ignores hits on the robot's OWN body unconditionally, regardless of
    geometry -- confirmed live this session: a full 360deg self-hit
    sweep (not just each mount's own facing direction, which is all the
    startup check verifies) found real self-hits on
    actuator_outer_cylinder_link at several angles even at the piston's
    REST position, ~0.08m out from several mount points. Picking a
    bigger clearance radius doesn't robustly fix this, since the piston/
    ball-joint stack's real shape changes as it lifts and articulates;
    unconditionally skipping past any hit on the robot's own body does,
    regardless of what shape it currently is."""
    world_bearing = chassis_yaw + body_angle_rad
    c, s = math.cos(world_bearing), math.sin(world_bearing)
    origin_x = chassis_x + CAMERA_MOUNT_RADIUS * c
    origin_y = chassis_y + CAMERA_MOUNT_RADIUS * s
    direction = (c, s, 0.0)
    traveled = 0.0
    remaining = max_range
    for _ in range(8):  # generous bound on self-hit skips per ray
        origin = (origin_x + direction[0] * traveled, origin_y + direction[1] * traveled, CAMERA_HEIGHT)
        hit = _physx_query.raycast_closest(origin, direction, remaining)
        if not hit["hit"]:
            return max_range
        body = str(hit.get("rigidBody", ""))
        if not body.startswith(ROBOT_PATH_PREFIX):
            # Real, external hit. Add CAMERA_MOUNT_RADIUS back so the
            # reported range means "distance from chassis center," the
            # convention the rest of this codebase (OBSTACLE_STOP_RANGE
            # etc.) already uses -- confirmed live this session: every
            # reference object read exactly 0.100m short and no other
            # amount, proving this is a pure, constant origin-offset
            # artifact, not measurement noise.
            return min(traveled + float(hit["distance"]) + CAMERA_MOUNT_RADIUS, max_range)
        # Self-hit -- skip past it and keep looking along the same ray.
        step = float(hit["distance"]) + SELF_HIT_RETRY_EPSILON
        traveled += step
        remaining -= step
        if remaining <= 0:
            return max_range
    return max_range


def synthesize_laser_ranges(chassis_x, chassis_y, chassis_yaw):
    """One range per fixed robot-frame angle (LASER_NUM_SAMPLES evenly
    spaced from -pi to pi), via a direct raycast fan from the robot's
    live world pose. Returned in increasing-azimuth order."""
    ranges = []
    for i in range(LASER_NUM_SAMPLES):
        angle = LASER_ANGLE_MIN + i * LASER_ANGLE_INC
        ranges.append(range_at_world_pose(chassis_x, chassis_y, chassis_yaw, angle))
    return ranges


def range_at_body_angle(chassis_x, chassis_y, chassis_yaw, angle):
    """Convenience alias for calibration/diagnostics: the range at one
    specific body-frame angle, without computing the full 360-sample scan."""
    return range_at_world_pose(chassis_x, chassis_y, chassis_yaw, angle)
