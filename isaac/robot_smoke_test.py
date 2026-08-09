"""Robot import smoke test -- loads the reference paper_robot_centered_actuator_v6
URDF (centered piston lift + ball-joint contact plate + skid-steer wheels) into
Isaac Sim, drops it onto a ground plane, and lets real PhysX gravity settle it.
Confirms the URDF -> USD -> PhysX articulation pipeline works end to end before
building any actual coupling-mechanism logic on top of it.

Uses the same low-VRAM MinimalRendering setup validated in
smoke_test_minimal_viewport.py -- this hardware can't run the default
ray-traced renderer (confirmed: it crashes on this GPU)."""
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

import os

import carb.settings
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.prims import Articulation
# Hide the built-in reference grid at y=0 -- separate from our own
# GroundPlane prim, this is Kit's viewport guide overlay. Live-toggleable
# in the GUI too (viewport eye icon -> Grid).
carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")

UsdPhysics.Scene.Define(stage, "/World/physicsScene")

UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(300)
distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
distant.CreateIntensityAttr(2500)
distant.CreateAngleAttr(2.0)
UsdGeom.Xformable(distant).AddRotateXYZOp().Set(Gf.Vec3f(-45, 35, 0))

GroundPlane("/World/GroundPlane", sizes=50, colors="gray", templates=None)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(
    REPO_ROOT, "gitignore", "paper_robot_centered_actuator_v6",
    "ros_gazebo", "paper_robot_centered_actuator_v6.urdf",
)

import_config = URDFImporterConfig(urdf_path=URDF_PATH, fix_base=False)
robot_usd_path = URDFImporter(import_config).import_urdf()
print(f"[robot smoke test] imported URDF -> {robot_usd_path}")

add_reference_to_stage(robot_usd_path, "/World/Robot")

# Start the robot a little above the floor so it visibly settles under
# gravity rather than spawning already interpenetrating the ground plane.
UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot")).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.2))


for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World/Robot"])

omni.timeline.get_timeline_interface().play()

robot = Articulation("/World/Robot/Geometry/base_link")
wheel_joints = robot.get_joint_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])

robot.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints)
robot.set_dof_velocity_targets(10, dof_indices=wheel_joints)

print("[robot smoke test] physics running -- robot should settle onto the ground plane under gravity")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
