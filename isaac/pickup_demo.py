"""Autonomous pick-up demo -- drives the robot straight forward (no steering
needed, the robot's spawn point in shelves1_with_collision.usd was moved to
sit directly in front of Cube_02 and face it) until it's under the object,
then raises the piston to lift it. Once the pick-up is done, control hands
off to the same keyboard teleop as robot_smoke_test.py so the object can be
driven away/tilted/lowered manually.

Uses the same low-VRAM MinimalRendering setup as robot_smoke_test.py."""
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

import os
import time

import carb.settings
import omni.timeline
import omni.appwindow
import carb.input
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from pxr import PhysxSchema

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

SHELF_USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "shelves1_with_collision.usd")
success, stage = stage_utils.open_stage(SHELF_USD_PATH)
print(f"[pickup demo] opened {SHELF_USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World/Robot"])

art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/Robot/Geometry/base_link"))
art_api.CreateSolverPositionIterationCountAttr(255)
art_api.CreateSolverVelocityIterationCountAttr(64)

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

robot = Articulation("/World/Robot/Geometry/base_link")

base_link_index = robot.get_link_indices(["base_link"])
robot.set_link_masses([40.0], link_indices=base_link_index)

wheel_joints = robot.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
robot.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints)
robot.set_dof_velocity_targets(0, dof_indices=wheel_joints)

tilt_joints = robot.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute", "ball_yaw_revolute"])
robot.set_dof_gains(stiffnesses=0, dampings=50.0, dof_indices=tilt_joints)
robot.set_dof_velocity_targets(0, dof_indices=tilt_joints)

piston_joint = robot.get_dof_indices(["centered_piston_prismatic_z"])
robot.set_dof_limits(lower=0.0, upper=2.0, dof_indices=piston_joint)
PISTON_STIFFNESS = 20000.0
PISTON_DAMPING = 1000.0
robot.set_dof_gains(stiffnesses=PISTON_STIFFNESS, dampings=PISTON_DAMPING, dof_indices=piston_joint)
piston_target = float(robot.get_dof_positions(dof_indices=piston_joint).numpy()[0, 0])
robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)

# --- Autonomous approach: robot spawn is on the shelf's SHORT side (the
# shelf rectangle is 2.16m wide x 0.58m deep -- entering face-first through
# the long/front side means driving straight into the full-width shelf floor
# panels; entering through the short side at y=-0.75 threads the gap between
# the closest pair of corner legs and lines up with the one Y-band across
# the shelf's depth that has no floor board at all (a real ~0.1m-wide
# construction gap in the shelf model, confirmed by measurement -- Cube_02
# happens to sag into it). Driving straight along X the whole way is clean,
# no piston pre-raise needed. Triggered by pressing P (see on_key_event
# below) rather than running automatically on launch.
TARGET_X = 0.164
Base_speed = 5.0
Piston_speed = 0.15
LIFT_TARGET = 0.2
pickup_done = False

def run_pickup_sequence():
    global piston_target, pickup_done
    print("[pickup demo] driving forward toward the shelf...")
    robot.set_dof_velocity_targets([Base_speed, Base_speed, Base_speed, Base_speed], dof_indices=wheel_joints)
    for _ in range(2000):
        pos, _ = robot.get_world_poses()
        x = float(pos.numpy()[0, 0])
        simulation_app.update()
        if x <= TARGET_X:
            print(f"[pickup demo] reached shelf, x={x:.3f}")
            break
    robot.set_dof_velocity_targets([0, 0, 0, 0], dof_indices=wheel_joints)
    for _ in range(30):
        simulation_app.update()

    # Lift: ramp the piston up slowly (same speed as the manual R/C control)
    # to a conservative height -- enough to lift Cube_02 clear of the shelf
    # without pushing it up into the tier above (tier gap starts at world
    # Z=0.45, object top after a 0.2m lift lands around Z=0.42, ~3cm margin).
    print("[pickup demo] lifting...")
    while piston_target < LIFT_TARGET:
        piston_target = min(LIFT_TARGET, piston_target + Piston_speed * (1 / 60))
        robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)
        simulation_app.update()
    print("[pickup demo] pick-up sequence complete -- back to manual control")
    pickup_done = True

# --- Same manual teleop as robot_smoke_test.py, plus P to trigger the
# autonomous pick-up sequence above (one-shot, like F).
input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World"])
        elif e.input == carb.input.KeyboardInput.P and not pickup_done:
            run_pickup_sequence()
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)
print("[pickup demo] press P to start the autonomous pick-up sequence")
print("[pickup demo] WASD drive, R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame whole map")

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

    robot.set_dof_velocity_targets([roll_vel, pitch_vel, yaw_vel], dof_indices=tilt_joints)

    simulation_app.update()

simulation_app.close()
