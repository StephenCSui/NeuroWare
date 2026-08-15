"""Low-VRAM viewport smoke test -- confirms Isaac Sim can hold an interactive
window open on this 4GB card using MinimalRendering (no ray tracing) instead
of the default RealTimePathTracing mode that likely caused the previous
SIGKILL right at renderer init."""
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,  # Diffuse/Glossy/Emission -- mode 0 is "No Rendering" (black by design)
    "width": 960,
    "height": 540,
    "window_width": 1000,
    "window_height": 640,
})

import carb.settings
import omni.usd
from pxr import Gf, UsdGeom, UsdLux
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
from isaacsim.core.experimental.objects import GroundPlane

# Hide the built-in reference grid at y=0 -- separate from our own
# GroundPlane prim, this is Kit's viewport guide overlay.
carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Cube.Define(stage, "/World/TestCube")

# Low-intensity dome light for ambient fill so nothing is ever fully black,
# plus an angled distant light as the real key light -- a dome alone lights
# every face equally, which is why the cube looked flat/edgeless before.
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(300)
distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
distant.CreateIntensityAttr(2500)
distant.CreateAngleAttr(2.0)
UsdGeom.Xformable(distant).AddRotateXYZOp().Set(Gf.Vec3f(-45, 35, 0))

GroundPlane("/World/GroundPlane", sizes=1000, colors="gray", templates=None)

# A couple of updates so the viewport/renderer is actually initialized
# before we ask it to frame anything.
for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World/TestCube"])

print("[smoke test] window should now be visible with a single lit cube, MinimalRendering mode")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
