"""Viewer for two_robot_two_shelf.usd -- two robots, two shelves (shelf 2 is
an empty delivery destination, offset +4m in Y from shelf 1). No 2-robot
coordination logic exists yet; this just lets Robot (the first instance)
be driven manually with the same controls as robot_smoke_test.py so the
duplicated layout can be inspected live. Robot2 sits idle for now.

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
from isaacsim.core.experimental.prims import Articulation
from pxr import PhysxSchema

carb.settings.get_settings().set("/app/viewport/grid/enabled", False)

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "two_robot_two_shelf.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[two robot demo] opened {USD_PATH} -> {success}")

for _ in range(5):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World"])

for path in ["/World/Robot/Geometry/base_link", "/World/Robot2/Geometry/base_link"]:
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
    art_api.CreateSolverPositionIterationCountAttr(255)
    art_api.CreateSolverVelocityIterationCountAttr(64)

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

robot = Articulation("/World/Robot/Geometry/base_link")
robot.set_link_masses([40.0], link_indices=robot.get_link_indices(["base_link"]))

wheel_joints = robot.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
robot.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints)
robot.set_dof_velocity_targets(0, dof_indices=wheel_joints)

tilt_joints = robot.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute", "ball_yaw_revolute"])
robot.set_dof_gains(stiffnesses=0, dampings=50.0, dof_indices=tilt_joints)
robot.set_dof_velocity_targets(0, dof_indices=tilt_joints)

piston_joint = robot.get_dof_indices(["centered_piston_prismatic_z"])
robot.set_dof_limits(lower=0.0, upper=2.0, dof_indices=piston_joint)
robot.set_dof_gains(stiffnesses=20000.0, dampings=1000.0, dof_indices=piston_joint)
piston_target = float(robot.get_dof_positions(dof_indices=piston_joint).numpy()[0, 0])
robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)

# Robot2 idles -- just settled/damped in place, not driven yet. No
# coordination logic between the two robots exists at this point.
robot2 = Articulation("/World/Robot2/Geometry/base_link")
robot2.set_link_masses([40.0], link_indices=robot2.get_link_indices(["base_link"]))
wheel_joints2 = robot2.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
robot2.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wheel_joints2)
robot2.set_dof_velocity_targets(0, dof_indices=wheel_joints2)
tilt_joints2 = robot2.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute", "ball_yaw_revolute"])
robot2.set_dof_gains(stiffnesses=0, dampings=50.0, dof_indices=tilt_joints2)
robot2.set_dof_velocity_targets(0, dof_indices=tilt_joints2)
piston_joint2 = robot2.get_dof_indices(["centered_piston_prismatic_z"])
robot2.set_dof_limits(lower=0.0, upper=2.0, dof_indices=piston_joint2)
robot2.set_dof_gains(stiffnesses=20000.0, dampings=1000.0, dof_indices=piston_joint2)

print("[two robot demo] WASD drive Robot, R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame whole map")
print("[two robot demo] Robot2 is idle -- no manual control or coordination wired up yet")

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World"])
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)

Base_speed = 5
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
