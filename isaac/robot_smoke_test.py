"""Robot + shelf + payload test scene -- opens the saved shelf/payload layout
(built by hand in the Isaac Sim GUI, then given physics via one-off scripts:
static collision on the shelf, dynamic rigid bodies on the payload objects)
together with the already-embedded robot, and adds keyboard-driven wheel
control so the robot can be driven up to the shelf to test real collision
with the payload objects.

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

import carb.settings
import omni.timeline
import omni.appwindow
import carb.input
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
# Hide the built-in reference grid at y=0 -- separate from the scene's own
# ground plane, this is Kit's viewport guide overlay. Live-toggleable
# in the GUI too (viewport eye icon -> Grid).
carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

# Not in the repo -- built by hand in the Isaac Sim GUI (Create > Shape),
# then given physics via one-off scripts: static collision on /World/Shelf,
# dynamic rigid bodies (CollisionAPI + RigidBodyAPI) on the 4 payload prims.
# Already contains the robot too (imported once, referenced at /World/Robot).
SHELF_USD_PATH = "/home/steph/isaac worlds/shelves1_with_collision.usd"
success, stage = stage_utils.open_stage(SHELF_USD_PATH)
print(f"[robot smoke test] opened {SHELF_USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World/Robot"])

omni.timeline.get_timeline_interface().play()

robot = Articulation("/World/Robot/Geometry/base_link")
# dof_indices= expects positions into dof_names (drivable DOFs only), not
# joint_names (which also includes fixed joints and shifts every index).
# get_dof_indices resolves against the correct space -- get_joint_indices
# does not, and silently produces indices that land on the wrong DOFs.
wheel_joints = robot.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)

robot.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints)
robot.set_dof_velocity_targets(0, dof_indices=wheel_joints)

# The piston and ball-joint stack have no drive at all by default (importer
# warned "actuator will be created without gain parameters") -- they're free
# to flop under gravity/motion with poorly-estimated inertia. Damping them
# to a zero velocity target stops that free flopping without locking them
# rigidly in place.
passive_joints = robot.get_dof_indices([
    "centered_piston_prismatic_z", "ball_roll_revolute", "ball_pitch_revolute", "ball_yaw_revolute",
])
robot.set_dof_gains(stiffnesses=0, dampings=50.0, dof_indices=passive_joints)
robot.set_dof_velocity_targets(0, dof_indices=passive_joints)

print("[robot smoke test] physics running -- robot should settle onto the ground plane under gravity")

Base_speed = 5
while simulation_app.is_running():
    forward = carb.input.KeyboardInput.W in held_keys
    backward = carb.input.KeyboardInput.S in held_keys
    left = carb.input.KeyboardInput.A in held_keys
    right = carb.input.KeyboardInput.D in held_keys

    speed = 0
    if forward:
        speed += Base_speed
    elif backward:
        speed -= Base_speed

    left_speed = 0
    right_speed = 0
    if left:
        left_speed -= Base_speed
        right_speed += Base_speed
    elif right:
        left_speed += Base_speed
        right_speed -= Base_speed

    robot.set_dof_velocity_targets([speed + left_speed, speed + right_speed, speed + left_speed, speed + right_speed], dof_indices=wheel_joints)

    simulation_app.update()

simulation_app.close()
