"""6-depth-camera VRAM scaling test -- follow-up to smoke_test_depth_sensor.py
(single camera, 882MiB total). Six cameras arranged in a ring around the
test cube, each pointed inward, to see whether VRAM cost scales linearly
with camera count or has meaningfully different per-camera marginal cost
(shared render pipeline setup might make camera #2-6 cheaper than #1, or
per-camera render products might compound faster than linear -- this
test exists to find out empirically rather than extrapolate).

LIVE window (not headless), MinimalRendering, same setup as the
single-camera test."""
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
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import UsdGeom, UsdLux
import omni.usd
import omni.timeline
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.sensors.camera import Camera

def gpu_mem():
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
        capture_output=True, text=True,
    ).stdout.strip()
    return int(out.splitlines()[0])

def lookat_quat(position, target):
    """Look-at quaternion for isaacsim Camera's default camera_axes="world"
    convention: +Z up, +X forward (confirmed by reading camera.py directly
    -- NOT USD's usual -Z forward, which was the bug in the first version
    of this test). Uses scipy's Rotation.from_matrix rather than a
    hand-rolled trace-based conversion -- the hand-rolled version produced
    NaN for camera #0 (pointed back along -X, a near-180-degree rotation
    from the matrix's reference frame), a known instability in that
    formula near trace ~= -1 that scipy's implementation handles
    correctly by picking a numerically stable branch."""
    fwd = target - position
    fwd = fwd / np.linalg.norm(fwd)
    world_up = np.array([0.0, 0.0, 1.0])
    z_axis = world_up - np.dot(world_up, fwd) * fwd
    if np.linalg.norm(z_axis) < 1e-6:
        z_axis = np.array([0.0, 1.0, 0.0])
    z_axis = z_axis / np.linalg.norm(z_axis)
    x_axis = fwd
    y_axis = np.cross(z_axis, x_axis)
    R = np.array([x_axis, y_axis, z_axis]).T
    xyzw = Rotation.from_matrix(R).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])

print(f"[depth x6] GPU mem after SimulationApp init: {gpu_mem()} MiB")

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Cube.Define(stage, "/World/TestCube")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(300)
GroundPlane("/World/GroundPlane", sizes=1000, colors="gray", templates=None)

for _ in range(5):
    simulation_app.update()

print(f"[depth x6] GPU mem after scene setup: {gpu_mem()} MiB")

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

# 6 cameras in a ring at radius 3, 60deg apart, all pointed inward at the
# cube -- a plausible stand-in for "one robot, 6 cameras around it" rather
# than 6 cameras all pointed the same direction (which wouldn't prove much).
target = np.array([0.0, 0.0, 0.5])
cameras = []
for i in range(6):
    angle = math.radians(60 * i)
    pos = np.array([3.0 * math.cos(angle), 3.0 * math.sin(angle), 1.0])
    quat = lookat_quat(pos, target)
    cam = Camera(
        prim_path=f"/World/Camera_{i}",
        position=pos,
        orientation=quat,
        frequency=20,
        resolution=(128, 128),
    )
    cam.initialize()
    cam.add_distance_to_camera_to_frame()
    cameras.append(cam)
    for _ in range(3):
        simulation_app.update()
    print(f"[depth x6] GPU mem after camera #{i+1}/6 created: {gpu_mem()} MiB")

for _ in range(10):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World"])

for i, cam in enumerate(cameras):
    depth = cam.get_depth()
    if depth is None:
        frame = cam.get_current_frame()
        depth = frame.get("distance_to_camera") if frame else None
    if depth is not None:
        print(f"[depth x6] camera #{i} depth: min={float(depth.min()):.3f} max={float(depth.max()):.3f} center={float(depth[64,64]):.3f}")
    else:
        print(f"[depth x6] camera #{i} depth: None")

print(f"[depth x6] GPU mem after all 6 cameras captured a frame: {gpu_mem()} MiB")

for _ in range(30):
    simulation_app.update()

print(f"[depth x6] GPU mem after 30 more ticks (all 6 live): {gpu_mem()} MiB")
print("[depth x6] window will stay open -- close it (or Ctrl-C the process) when done looking")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
