"""Low-VRAM depth-sensor smoke test -- isolated check of whether a small
depth camera (128x128, depth-only, no RGB/material shading) can run on
this 4GB card without blowing VRAM, separately from the main viewport's
already-proven MinimalRendering setup (see smoke_test_minimal_viewport.py).
A camera sensor is a SEPARATE render product from the main viewport, so
MinimalRendering there doesn't guarantee this is cheap too -- this test
exists specifically to find out, empirically, before building any real
sensing pipeline on top of it.

LIVE window (not headless) with MinimalRendering, matching the actual
target combination (main viewport + depth sensor running together) --
the first pass of this test ran fully headless (no viewport render
product at all), which likely undersold the real cost. Prints GPU
memory used before/after camera creation and after capturing a frame."""
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

import subprocess
import numpy as np
from pxr import UsdGeom, UsdLux, Gf
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

print(f"[depth smoke test] GPU mem after SimulationApp init: {gpu_mem()} MiB")

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Cube.Define(stage, "/World/TestCube")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(300)
GroundPlane("/World/GroundPlane", sizes=1000, colors="gray", templates=None)

for _ in range(5):
    simulation_app.update()

print(f"[depth smoke test] GPU mem after scene setup: {gpu_mem()} MiB")

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

# 128x128, depth-only -- deliberately small, matching what a basic
# obstacle-detection check would actually need, not camera-quality output.
# Orientation: look-at quaternion aiming at the cube's center (0,0,0).
# set_world_pose's default camera_axes="world" convention is (+Z up,
# +X forward) -- NOT USD's usual (-Z forward) -- confirmed by reading
# camera.py directly. The previous two attempts both assumed -Z forward
# (first with no orientation at all, then a 90deg-about-X guess that
# still grazed the wrong direction under this convention), which is why
# the camera kept pointing at "completely the wrong area" despite the
# depth stats looking superficially plausible.
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0.0, -3.0, 1.0]),
    orientation=np.array([0.6979762, -0.1132660, 0.1132660, 0.6979762]),
    frequency=20,
    resolution=(128, 128),
)
camera.initialize()
camera.add_distance_to_camera_to_frame()
print(f"[depth smoke test] GPU mem after Camera() created + depth annotator enabled: {gpu_mem()} MiB")

for _ in range(10):
    simulation_app.update()

# frame the whole scene so the cube AND the camera prim's own gizmo icon
# are actually in view -- without this, the default Perspective viewport
# has no reason to be pointed anywhere near /World/Camera at all.
frame_viewport_prims(get_active_viewport(), ["/World"])

depth = camera.get_depth()
if depth is None:
    frame = camera.get_current_frame()
    depth = frame.get("distance_to_camera") if frame else None
print(f"[depth smoke test] depth frame shape: {None if depth is None else depth.shape}, dtype: {None if depth is None else depth.dtype}")
if depth is not None:
    print(f"[depth smoke test] depth sample values: min={float(depth.min()):.3f} max={float(depth.max()):.3f} center={float(depth[64,64]):.3f}")
print(f"[depth smoke test] GPU mem after first depth capture: {gpu_mem()} MiB")

for _ in range(30):
    simulation_app.update()

print(f"[depth smoke test] GPU mem after 30 more ticks: {gpu_mem()} MiB")
print("[depth smoke test] window will stay open -- close it (or Ctrl-C the process) when done looking")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
