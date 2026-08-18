"""Reusable, scene-agnostic visual debug indicators for Isaac Sim.

Every prim created here is pure visual guide geometry -- no collision, no
rigid body -- grouped under a single root Xform (DEBUG_VIS_ROOT) with one
sub-group per indicator category, so the whole thing (or just one
category) can be shown/hidden via the Stage hierarchy's native per-prim
visibility eye icon. No custom UI needed.

Deliberately never meant to be saved into a USD file: any script using
this module should call clear_debug_vis() before init_debug_vis() at
startup, so a stray GUI save can't bake a stale copy in and break the
next boot the way a runtime-created OmniGraph node once did (see
nav2_bridge_robot1.py's GRAPH_PATH handling).

Written to be reused as-is by future scripts driving other USD worlds
(e.g. a planned camera-calibration scene) -- takes an explicit `stage`
and explicit WORLD-frame coordinates everywhere, no assumptions about
odom frames, robot paths, or any particular scene's layout. A caller
that has odom-frame data (this project's convention) is responsible for
converting to world before calling in here.

Every function is safe to call speculatively -- each one catches its own
exceptions, logs a one-line warning, and returns None on failure rather
than raising, so a missing prim or bad input never takes down whatever
main loop is calling it.
"""
from pxr import Usd, UsdGeom, Gf
import math

DEBUG_VIS_ROOT = "/World/DebugVis"
CATEGORIES = ("CameraFOV", "StopRange", "Waypoints", "ExitEntry")


def clear_debug_vis(stage, root_path=DEBUG_VIS_ROOT):
    """Removes any leftover debug-vis prims (e.g. baked in by a stray
    GUI save) before rebuilding fresh. Safe to call even if nothing
    exists yet."""
    try:
        prim = stage.GetPrimAtPath(root_path)
        if prim and prim.IsValid():
            stage.RemovePrim(root_path)
    except Exception as e:
        print(f"[debug_viz] WARNING: clear_debug_vis failed ({e}) -- continuing")


def _ensure_group(stage, path):
    try:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            UsdGeom.Xform.Define(stage, path)
        return stage.GetPrimAtPath(path)
    except Exception as e:
        print(f"[debug_viz] WARNING: _ensure_group({path}) failed ({e}) -- continuing")
        return None


def init_debug_vis(stage, root_path=DEBUG_VIS_ROOT):
    """Call once at startup, after clear_debug_vis(). Creates the root
    group and its category sub-groups so they're visible in the Stage
    hierarchy immediately, even before anything's been drawn into them."""
    try:
        _ensure_group(stage, root_path)
        for sub in CATEGORIES:
            _ensure_group(stage, f"{root_path}/{sub}")
    except Exception as e:
        print(f"[debug_viz] WARNING: init_debug_vis failed ({e}) -- continuing")


def clear_group(stage, group_path):
    """Removes every child of a category group (e.g. before redrawing
    the waypoint list for a new plan), leaving the group itself intact."""
    try:
        prim = stage.GetPrimAtPath(group_path)
        if not prim or not prim.IsValid():
            return
        for child in list(prim.GetChildren()):
            stage.RemovePrim(child.GetPath())
    except Exception as e:
        print(f"[debug_viz] WARNING: clear_group({group_path}) failed ({e}) -- continuing")


def _point_color_and_size(kind):
    return {
        "waypoint": (Gf.Vec3f(1.0, 0.85, 0.0), 0.06),   # yellow, small
        "exit": (Gf.Vec3f(1.0, 0.2, 0.2), 0.10),        # red
        "entry": (Gf.Vec3f(0.2, 1.0, 0.3), 0.10),       # green
        "staging": (Gf.Vec3f(0.2, 0.6, 1.0), 0.10),     # blue
    }.get(kind, (Gf.Vec3f(1.0, 1.0, 1.0), 0.08))


def create_point_marker(stage, path, x, y, z=0.15, kind="waypoint", label=""):
    """Small pure-visual sphere at a given WORLD (x, y, z). `kind`
    selects a color/size preset (see _point_color_and_size); anything
    else falls back to a neutral white marker."""
    try:
        color, radius = _point_color_and_size(kind)
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(radius)
        sphere.CreateDisplayColorAttr([color])
        xf = UsdGeom.Xformable(sphere)
        xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
        if label:
            sphere.GetPrim().SetMetadata("comment", label)
        return sphere.GetPrim()
    except Exception as e:
        print(f"[debug_viz] WARNING: create_point_marker({path}) failed ({e}) -- continuing")
        return None


def draw_waypoints(stage, points, group_path=f"{DEBUG_VIS_ROOT}/Waypoints"):
    """Replaces the current contents of the Waypoints group with one
    marker per (x, y) WORLD-frame point, in order. Cheap to call on
    every new plan -- clears and redraws rather than diffing."""
    try:
        clear_group(stage, group_path)
        for i, (x, y) in enumerate(points):
            create_point_marker(stage, f"{group_path}/wp_{i}", x, y, kind="waypoint", label=f"waypoint {i}")
    except Exception as e:
        print(f"[debug_viz] WARNING: draw_waypoints failed ({e}) -- continuing")


def draw_exit_entry_markers(stage, labeled_points, group_path=f"{DEBUG_VIS_ROOT}/ExitEntry"):
    """labeled_points: dict of label -> (x, y) in WORLD frame, e.g.
    {"shelf1_exit": (x, y), "shelf2_staging": (x, y), "shelf2_entry": (x, y)}.
    Redraws the whole group each call (small, infrequent updates)."""
    try:
        clear_group(stage, group_path)
        for label, (x, y) in labeled_points.items():
            kind = "exit" if "exit" in label else ("staging" if "staging" in label else "entry")
            create_point_marker(stage, f"{group_path}/{label}", x, y, kind=kind, label=label)
    except Exception as e:
        print(f"[debug_viz] WARNING: draw_exit_entry_markers failed ({e}) -- continuing")


def create_stop_range_circle(stage, radius, num_segments=32,
                              group_path=f"{DEBUG_VIS_ROOT}/StopRange"):
    """Draws a flat ring of `radius` meters as a BasisCurves loop in the
    XY plane, at the DEBUG_VIS_ROOT-relative path (not truly USD-parented
    under the robot, so it stays under the shared toggle root) --
    replaces any existing ring under this group first. Only needs
    redrawing when the radius itself changes (e.g. carrying state
    toggles). Call move_prim_to() every tick to keep it centered on the
    robot's current position, same as any other free-standing marker."""
    try:
        clear_group(stage, group_path)
        path = f"{group_path}/ring"
        curve = UsdGeom.BasisCurves.Define(stage, path)
        pts = [
            Gf.Vec3f(radius * math.cos(2 * math.pi * i / num_segments),
                     radius * math.sin(2 * math.pi * i / num_segments), 0.02)
            for i in range(num_segments + 1)
        ]
        curve.CreatePointsAttr(pts)
        curve.CreateCurveVertexCountsAttr([len(pts)])
        curve.CreateTypeAttr("linear")
        curve.CreateWrapAttr("nonperiodic")
        curve.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.4, 0.0)])
        curve.CreateWidthsAttr([0.01] * len(pts))
        return curve.GetPrim()
    except Exception as e:
        print(f"[debug_viz] WARNING: create_stop_range_circle failed ({e}) -- continuing")
        return None


def move_prim_to(stage, path, x, y, z=0.02):
    """Overwrites a prim's translate op to a new WORLD position -- used
    to keep the stop-range ring following the robot each tick without
    needing true USD parenting under a physics-driven prim."""
    try:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(float(x), float(y), float(z)))
        else:
            xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    except Exception as e:
        print(f"[debug_viz] WARNING: move_prim_to({path}) failed ({e}) -- continuing")


def create_camera_fov_cone(stage, parent_path, name, local_pos, local_yaw_deg, fov_deg, range_m):
    """Draws two boundary rays (+ a center ray) out to `range_m`, splayed
    at +-fov_deg/2 around the given LOCAL yaw (degrees, same convention
    as everywhere else in this project -- 0 = parent's local +X,
    increasing counterclockwise). Lives as a CHILD of `parent_path` (the
    robot's base_link, not the camera prim itself) with an explicitly
    authored translate+rotateZ matching the camera's own intended mount
    pose (`local_pos`, `local_yaw_deg`) -- so it inherits base_link's
    live world transform for free (moves/turns with the robot) without
    depending on the camera prim's own stored orientation, which Isaac's
    Camera class internally remaps to USD's native camera convention
    (confirmed live: parenting under the camera prim produced a visibly
    tilted cone -- the sensing itself is unaffected since that goes
    through Isaac's real frustum/render pipeline, not this raw-axis
    assumption, but the debug visual was wrong)."""
    try:
        group_path = f"{parent_path}/{name}"
        group = UsdGeom.Xform.Define(stage, group_path)
        gxf = UsdGeom.Xformable(group)
        gxf.AddTranslateOp().Set(Gf.Vec3d(float(local_pos[0]), float(local_pos[1]), float(local_pos[2])))
        gxf.AddRotateZOp().Set(float(local_yaw_deg))

        path = f"{group_path}/FOVCone"
        curve = UsdGeom.BasisCurves.Define(stage, path)
        half = math.radians(fov_deg) / 2.0

        def _ray(angle):
            return Gf.Vec3f(range_m * math.cos(angle), range_m * math.sin(angle), 0.0)

        origin = Gf.Vec3f(0.0, 0.0, 0.0)
        pts = [origin, _ray(-half), origin, _ray(half), origin, _ray(0.0)]
        curve.CreatePointsAttr(pts)
        curve.CreateCurveVertexCountsAttr([2, 2, 2])
        curve.CreateTypeAttr("linear")
        curve.CreateWrapAttr("nonperiodic")
        curve.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.8, 1.0)])
        curve.CreateWidthsAttr([0.005] * len(pts))
        return curve.GetPrim()
    except Exception as e:
        print(f"[debug_viz] WARNING: create_camera_fov_cone({parent_path}/{name}) failed ({e}) -- continuing")
        return None
