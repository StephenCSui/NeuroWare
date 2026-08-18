"""One-off builder: creates isaac/test/camera_calibration.usd, a
stripped-down copy of two_robot_two_shelf.usd containing only the robot
(same asset, same physics setup -- copied via Sdf layer export + prim
removal, not rebuilt from scratch) plus 4 reference objects at precisely
known bearings/distances from the robot, for characterizing the corner
depth cameras' measurement bias against a ground-truth reference LiDAR.

Ground-truth object positions are computed from the robot's REAL resolved
world pose in this file (queried directly, not assumed) and written out
to camera_calibration_reference.json alongside the USD, so
camera_calibration.py never has to re-derive or guess them.

Run once via Isaac's python.sh (needs pxr/Usd, same convention as every
other one-off scene-editing script this project has used):
    PYTHONPATH= AMENT_PREFIX_PATH= COLCON_PREFIX_PATH= HEADLESS=1 \\
        /home/steph/isaacsim/python.sh test/build_camera_calibration_world.py
"""
import json
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": os.environ.get("HEADLESS", "1") == "1",
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
})

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import isaacsim.core.experimental.utils.stage as stage_utils

SRC_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "two_robot_two_shelf.usd")
DST_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.usd")
REFERENCE_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration_reference.json")

success, stage = stage_utils.open_stage(SRC_PATH)
print(f"[build_camera_calibration_world] opened {SRC_PATH} -> {success}")
for _ in range(5):
    simulation_app.update()

# Strip everything except the robot + the basic environment prims
# (ground plane, lights, physics scene). Runtime-only prims
# (DebugVis, Ros2NavGraph) shouldn't exist in the on-disk file per this
# project's established discipline, but check anyway rather than assume.
KEEP = {"/World/physicsScene", "/World/DomeLight", "/World/KeyLight",
        "/World/GroundPlane", "/World/Robot"}
world = stage.GetPrimAtPath("/World")
for child in list(world.GetChildren()):
    path = str(child.GetPath())
    if path not in KEEP:
        print(f"[build_camera_calibration_world] removing {path}")
        stage.RemovePrim(child.GetPath())
# Root-level payload objects (siblings of /World, not under it, per this
# scene's own layout -- confirmed via SCENE_DUMP_FILE queries in earlier
# sessions, not assumed).
for name in ("Cylinder", "Cube", "Cube_01", "Cube_02", "Cube_03"):
    p = stage.GetPrimAtPath(f"/{name}")
    if p and p.IsValid():
        print(f"[build_camera_calibration_world] removing /{name}")
        stage.RemovePrim(p.GetPath())

# Real resolved world pose of the robot's base_link -- queried directly
# via XformCache, not assumed. No physics stepping needed for a static
# read of the authored transform stack.
ROBOT_PATH = "/World/Robot/Geometry/base_link"
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
world_mat = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT_PATH))
translation = world_mat.ExtractTranslation()
robot_x, robot_y = float(translation[0]), float(translation[1])
# Yaw from the matrix's rotation (Z-up, standard 2D heading extraction).
rot = world_mat.ExtractRotationMatrix()
robot_yaw = math.atan2(rot[1][0], rot[0][0])
print(f"[build_camera_calibration_world] robot base_link at world ({robot_x:.4f}, {robot_y:.4f}), yaw={math.degrees(robot_yaw):.2f}deg")

# Reference objects: label, bearing (deg, relative to robot forward),
# distance (m), and a builder function taking (path, world_x, world_y).
# Bearings chosen to hit a spread of the corner cameras' actual coverage
# (FL/FR centered at +-45deg, RL/RR at 135/-135deg): 0deg sits in the
# FL/FR overlap zone (tests the reconciliation logic), +-45deg are each
# dead-center in one camera's FOV, and 100deg is near a FOV boundary.
def _make_wall(stage, path, x, y, yaw):
    # Flat box, long axis perpendicular to the ray from the robot so it
    # presents a clean flat face -- rotated to face back toward the robot.
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.25))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(0.05, 1.0, 0.5))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()

def _make_cylinder(stage, path, x, y, radius):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(0.5)
    cyl.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.25))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()

def _make_corner_edge(stage, path, x, y, yaw):
    # Originally two thin (0.05m) offset planks forming an L. Live-verified
    # this session via direct PhysX raycast: with the group rotated so a
    # corner faces the robot, the plank whose LONG axis points back toward
    # the robot presents an almost edge-on, near-zero cross-section to any
    # ray from that direction -- its enclosing AABB looked large enough to
    # hit, but the true (diagonally rotated) box was a sliver the ray
    # missed entirely, confirmed by a raycast reporting no hit at all.
    # Replaced with a single solid cube, still yawed 45deg so an EDGE (not
    # a flat face) points at the robot -- tests a non-flat/angled surface
    # like the original intent, but with real cross-section from any
    # approach angle instead of depending on grazing a paper-thin plank.
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.25))
    xf.AddRotateZOp().Set(math.degrees(yaw) + 45.0)
    xf.AddScaleOp().Set(Gf.Vec3f(0.25, 0.25, 0.5))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()

def _make_thin_post(stage, path, x, y):
    # Radius widened 0.03 -> 0.06 this session -- 0.03m subtends only
    # ~3.4deg at 1.0m, marginal enough that a real depth-camera-column or
    # single-raycast sample can miss it depending on exact alignment
    # (confirmed: it was undetected in testing). Still a comparatively
    # thin target, just no longer right at the edge of detectability.
    post = UsdGeom.Cylinder.Define(stage, path)
    post.CreateRadiusAttr(0.06)
    post.CreateHeightAttr(0.6)
    post.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(post)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.3))
    UsdPhysics.CollisionAPI.Apply(post.GetPrim())
    return post.GetPrim()

REFERENCE_OBJECTS = [
    {"label": "wall", "bearing_deg": 0.0, "distance_m": 2.0, "kind": "wall"},
    {"label": "corner_edge", "bearing_deg": -45.0, "distance_m": 1.5, "kind": "corner_edge"},
    {"label": "cylinder", "bearing_deg": 45.0, "distance_m": 2.5, "kind": "cylinder"},
    {"label": "thin_post", "bearing_deg": 100.0, "distance_m": 1.0, "kind": "thin_post"},
]

reference_records = []
for obj in REFERENCE_OBJECTS:
    bearing_rad = math.radians(obj["bearing_deg"])
    world_bearing = robot_yaw + bearing_rad
    x = robot_x + obj["distance_m"] * math.cos(world_bearing)
    y = robot_y + obj["distance_m"] * math.sin(world_bearing)
    path = f"/World/RefObjects/{obj['label']}"
    if obj["kind"] == "wall":
        # face the wall back toward the robot: its own +Y (long axis) is
        # perpendicular to the ray, so rotate so local +X points at the robot.
        _make_wall(stage, path, x, y, world_bearing + math.pi)
    elif obj["kind"] == "cylinder":
        _make_cylinder(stage, path, x, y, radius=0.15)
    elif obj["kind"] == "corner_edge":
        _make_corner_edge(stage, path, x, y, world_bearing + math.pi)
    elif obj["kind"] == "thin_post":
        _make_thin_post(stage, path, x, y)
    reference_records.append({
        "label": obj["label"], "kind": obj["kind"],
        "bearing_deg_from_robot_forward": obj["bearing_deg"],
        "true_distance_m": obj["distance_m"],
        "world_x": x, "world_y": y,
    })
    print(f"[build_camera_calibration_world] placed {obj['label']} ({obj['kind']}) at "
          f"world ({x:.3f}, {y:.3f}), bearing {obj['bearing_deg']}deg, distance {obj['distance_m']}m")

for _ in range(5):
    simulation_app.update()

stage.GetRootLayer().Export(DST_PATH)
print(f"[build_camera_calibration_world] exported {DST_PATH}")

with open(REFERENCE_JSON_PATH, "w") as f:
    json.dump({
        "robot_world_x": robot_x, "robot_world_y": robot_y, "robot_world_yaw_deg": math.degrees(robot_yaw),
        "objects": reference_records,
    }, f, indent=2)
print(f"[build_camera_calibration_world] wrote {REFERENCE_JSON_PATH}")

simulation_app.close()
