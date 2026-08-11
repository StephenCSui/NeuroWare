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

import os
import time

import carb.settings
import omni.timeline
import omni.appwindow
import carb.input
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from pxr import PhysxSchema
# Hide the built-in reference grid at y=0 -- separate from the scene's own
# ground plane, this is Kit's viewport guide overlay. Live-toggleable
# in the GUI too (viewport eye icon -> Grid).
carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

# Built by hand in the Isaac Sim GUI (Create > Shape), then given physics via
# one-off scripts: static collision on /World/Shelf, dynamic rigid bodies
# (CollisionAPI + RigidBodyAPI) on the 4 payload prims. Already contains the
# robot too (imported once, referenced at /World/Robot).
SHELF_USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "shelves1_with_collision.usd")
success, stage = stage_utils.open_stage(SHELF_USD_PATH)
print(f"[robot smoke test] opened {SHELF_USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World/Robot"])

# base_link (40kg, see below) vs each wheel link (~0.14kg) is a ~280:1 mass
# ratio -- PhysX's default solver iteration count can't converge a joint
# constraint across a ratio that extreme, so wheels commanded to turn just
# sit at literally 0 velocity no matter how much drive torque is requested
# (confirmed: 5x more damping made zero difference). Raising the
# articulation's own solver iteration counts (not the drive torque) is what
# actually fixes it -- confirmed empirically to restore full turning.
art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/Robot/Geometry/base_link"))
art_api.CreateSolverPositionIterationCountAttr(255)
art_api.CreateSolverVelocityIterationCountAttr(64)

omni.timeline.get_timeline_interface().play()
# get_dof_positions below is tensor-backend-only (unlike the setters, which
# fall back to writing USD directly) -- it needs a few physics steps after
# play() before the physics tensor view is valid.
for _ in range(5):
    simulation_app.update()

robot = Articulation("/World/Robot/Geometry/base_link")

# base_link is ~5kg out of ~9.75kg total robot mass, while a single payload
# object can be up to 16kg -- heavier than the whole robot. Bumping base_link
# mass (its own low chassis origin, not the elevated piston/ball stack) so it
# dominates the mass budget raises tip-resistance without raising the CoG.
base_link_index = robot.get_link_indices(["base_link"])
robot.set_link_masses([40.0], link_indices=base_link_index)

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
        if e.input == carb.input.KeyboardInput.F:
            # one-shot camera action (not a held drive key) -- reframes on
            # the whole scene instead of just the robot.
            frame_viewport_prims(get_active_viewport(), ["/World"])
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)

robot.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints)
robot.set_dof_velocity_targets(0, dof_indices=wheel_joints)

# The ball-joint tilt stack has no drive at all by default (importer warned
# "actuator will be created without gain parameters") -- free to flop under
# gravity/motion with poorly-estimated inertia. Damping to a zero velocity
# target stops that free flopping without locking it rigidly in place.
# Kept simple (velocity/damping-only, no position holding) for now -- only
# the piston needs to actually hold a load right now.
tilt_joints = robot.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute", "ball_yaw_revolute"])
robot.set_dof_gains(stiffnesses=0, dampings=50.0, dof_indices=tilt_joints)
robot.set_dof_velocity_targets(0, dof_indices=tilt_joints)

piston_joint = robot.get_dof_indices(["centered_piston_prismatic_z"])
# Real cylinder spec caps this joint at lowerLimit=0m (robot's own lower
# chassis -- already correct), upperLimit=0.25m -- raised well past the
# physical cylinder's real reach so testing/tuning shelf heights isn't
# capped by that hardware spec.
robot.set_dof_limits(lower=0.0, upper=2.0, dof_indices=piston_joint)

# Damping-only (stiffness=0, like the tilt joints above) can't hold a static
# load against gravity -- it's a pure velocity drive, so even at a zero
# velocity target the ~4kg hanging off the piston (up to ~20kg carrying the
# heaviest payload) makes it sink at a slow terminal velocity instead of
# actually floating. Real stiffness turns this into a position-holding
# drive -- it floats at whatever height it's left at instead of drifting.
# Gains are a first cut (not derived from a real stability analysis), tune
# live if it feels too soft or too stiff.
PISTON_STIFFNESS = 20000.0
PISTON_DAMPING = 1000.0
robot.set_dof_gains(stiffnesses=PISTON_STIFFNESS, dampings=PISTON_DAMPING, dof_indices=piston_joint)
piston_target = float(robot.get_dof_positions(dof_indices=piston_joint).numpy()[0, 0])
robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)

print("[robot smoke test] physics running -- robot should settle onto the ground plane under gravity")

Base_speed = 5
# Piston (Z) and ball-joint (roll/pitch/yaw) teleop -- R/C raise/lower the
# piston, arrow keys tilt (up/down = pitch, left/right = roll), Q/E yaw.
# F is not a drive key -- it's a one-shot "frame the whole map" camera
# action, handled in on_key_event above.
Piston_speed = 0.15
Tilt_speed = 2
last_time = time.perf_counter()
while simulation_app.is_running():
    now = time.perf_counter()
    dt = now - last_time
    last_time = now

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

    piston_up = carb.input.KeyboardInput.R in held_keys
    piston_down = carb.input.KeyboardInput.C in held_keys
    roll_left = carb.input.KeyboardInput.LEFT in held_keys
    roll_right = carb.input.KeyboardInput.RIGHT in held_keys
    pitch_up = carb.input.KeyboardInput.UP in held_keys
    pitch_down = carb.input.KeyboardInput.DOWN in held_keys
    yaw_left = carb.input.KeyboardInput.Q in held_keys
    yaw_right = carb.input.KeyboardInput.E in held_keys

    # Position-held, not velocity-driven -- moves the target while a key is
    # down, holds it (floats) via the piston's stiffness otherwise.
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

    yaw_vel = 0
    if yaw_left:
        yaw_vel -= Tilt_speed
    elif yaw_right:
        yaw_vel += Tilt_speed

    # order matches tilt_joints: [ball_roll, ball_pitch, ball_yaw]
    robot.set_dof_velocity_targets([roll_vel, pitch_vel, yaw_vel], dof_indices=tilt_joints)

    simulation_app.update()

simulation_app.close()
