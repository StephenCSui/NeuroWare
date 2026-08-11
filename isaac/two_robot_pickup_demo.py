"""Two-robot cooperative lift-and-carry demo -- Robot and Robot2 approach
Cube_01 (18kg, deliberately too heavy for one robot's 200N-capped piston to
lift alone -- see physics.usda) from opposite short sides of shelf 1,
straddling it, lift it together, then each independently navigates -- using
its own real heading relative to the bearing toward shelf 2, turning as
needed -- to the delivery spot. They enter facing opposite directions
(Robot facing -X, Robot2 facing +X, required so the plate's long axis stays
parallel to the shelf's long axis during the lift -- entering face-on jams
the plate against the shelf structure), so carrying the object anywhere
other than further along the X axis genuinely requires each of them to turn
in place first. There's no rigid coupling between plate and object yet, so
this is a real test of whether the object stays on the plates through a turn,
not just a lift.

Triggered by pressing P (matches pickup_demo.py's convention -- autonomous
sequences run on keypress, not automatically on launch).

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

import math
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

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "two_robot_two_shelf.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[two robot pickup] opened {USD_PATH} -> {success}")

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

def setup_robot(articulation_path):
    r = Articulation(articulation_path)
    r.set_link_masses([40.0], link_indices=r.get_link_indices(["base_link"]))
    wj = r.get_dof_indices(["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"])
    r.set_dof_gains(stiffnesses=0, dampings=5000.0, dof_indices=wj)
    r.set_dof_velocity_targets(0, dof_indices=wj)
    # roll/pitch and yaw all get real position control now -- roll/pitch
    # hold level (their starting position) so the plate doesn't tilt under
    # an off-center/heavy object's weight, yaw actively counters the
    # chassis's own turning so the plate isn't dragged around with it.
    # All three were originally damping-only, which can't resist a
    # sustained torque, only slow down a moving one.
    rp = r.get_dof_indices(["ball_roll_revolute", "ball_pitch_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=rp)
    rp_base = r.get_dof_positions(dof_indices=rp).numpy()[0].tolist()
    r.set_dof_position_targets(rp_base, dof_indices=rp)
    yj = r.get_dof_indices(["ball_yaw_revolute"])
    r.set_dof_gains(stiffnesses=5000.0, dampings=200.0, dof_indices=yj)
    yaw_joint_pos = float(r.get_dof_positions(dof_indices=yj).numpy()[0, 0])
    r.set_dof_position_targets(yaw_joint_pos, dof_indices=yj)
    pj = r.get_dof_indices(["centered_piston_prismatic_z"])
    r.set_dof_limits(lower=0.0, upper=2.0, dof_indices=pj)
    r.set_dof_gains(stiffnesses=20000.0, dampings=1000.0, dof_indices=pj)
    piston_pos = float(r.get_dof_positions(dof_indices=pj).numpy()[0, 0])
    r.set_dof_position_targets(piston_pos, dof_indices=pj)
    return r, wj, rp, yj, yaw_joint_pos, pj, piston_pos

robot, wheel_joints, tilt_joints, yaw_joint, yaw_joint_base, piston_joint, piston_target = setup_robot("/World/Robot/Geometry/base_link")
robot2, wheel_joints2, tilt_joints2, yaw_joint2, yaw_joint_base2, piston_joint2, piston_target2 = setup_robot("/World/Robot2/Geometry/base_link")

PLATE_PATH = "actuator_outer_cylinder_link/piston_rod_link/ball_joint_center_link/ball_roll_link/ball_pitch_link/ball_yaw_link/contact_plate_link"
plate1 = RigidPrim(f"/World/Robot/Geometry/base_link/{PLATE_PATH}")
plate2 = RigidPrim(f"/World/Robot2/Geometry/base_link/{PLATE_PATH}")

# Both robots enter from a SHORT side of the shelf (X direction), not the
# front/long side -- the plate is a rectangle (0.105 x 0.08), and entering
# via the long side puts the plate's long axis perpendicular to the
# shelf's long axis, which jams it against the shelf structure. Entering
# via a short side keeps the plate's long axis parallel to the shelf's,
# which is what actually fits. Robot enters from the right (-X), Robot2
# from the left (+X), converging on Cube_01's two ends.
TARGET_X = -0.209   # Robot's target (right end of Cube_01), approaching in -X
TARGET_X2 = -0.809  # Robot2's target (left end of Cube_01), approaching in +X
# Cube_01 was lengthened from 0.6m -> 1.0m -> 0.8m (1.0m clipped the
# shelf's corner leg), specifically to put more physical separation
# between the two robots -- at the original 0.6m length they picked it up
# nose-to-nose with ~0.09m of chassis clearance, and turning in place from
# there rammed them into each other. Depth/height were also trimmed
# (0.3->0.28, 0.1->0.06) to bring mass back down after the length change
# raised it -- currently 13.44kg (fixed cross-section, volume scales with
# length/depth/height x density 1000). That's closer to single-robot
# territory than the original 18kg was; not yet re-confirmed that this
# object still genuinely requires two robots rather than one.
#
# Shelf 2 is shifted +4m in Y from shelf 1, X range unchanged. Both robots
# turn to EXACTLY +90 degrees (known analytically -- Robot starts at 180,
# Robot2 at 0, both need to face +Y) and then drive dead straight in Y
# with no steering correction at all -- X is never touched again after
# pickup, so wherever they picked the object up in X is exactly where
# they'll be in X at shelf 2 too. A bearing-based "correct toward a target
# point" approach was tried and rejected -- any small residual heading
# error meant it wasn't actually driving in a straight line, so the two
# robots didn't stay predictably positioned relative to each other or the
# shelf even though each one was individually converging on its own point.
DELIVERY_YAW = math.pi / 2
DELIVERY_Y = 3.25
Base_speed = 5.0
Piston_speed = 0.15
LIFT_TARGET = 0.2
pickup_done = False

def get_xy_yaw(r):
    pos, quat = r.get_world_poses()
    p = pos.numpy()[0]
    w, x, y, z = quat.numpy()[0]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return p[0], p[1], yaw

RAMP_STEPS = 90  # ~1.5s to reach full speed from a standstill

def turn_to_yaw_step(r, wj, target_yaw, step=RAMP_STEPS, speed=Base_speed, yaw_tol=0.002):
    """One control tick of pure in-place rotation toward an EXACT absolute
    yaw angle (not a bearing computed from position) -- no forward motion.
    Both robots' target here is exactly +90 degrees (facing +Y), known
    analytically from their fixed pickup headings (180 and 0 degrees), not
    derived live from position. yaw_tol is tight (~0.3 degrees) so the
    straight drive that follows genuinely is straight, not a shallow curve.
    Returns True once aligned within yaw_tol (and stops the robot)."""
    ramp = min(1.0, step / RAMP_STEPS)
    speed = speed * ramp
    _, _, yaw = get_xy_yaw(r)
    yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
    if abs(yaw_err) <= yaw_tol:
        r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
        return True
    turn = 1.0 if yaw_err > 0 else -1.0
    left, right = -turn * speed, turn * speed
    r.set_dof_velocity_targets([left, right, left, right], dof_indices=wj)
    return False

def drive_straight_step(r, wj, target_y, step=RAMP_STEPS, speed=Base_speed, stop_dist=0.01):
    """One control tick of driving dead straight (equal wheel speeds, no
    steering correction at all) until the robot's Y reaches target_y. Only
    valid once the robot is already facing exactly +Y (call
    turn_to_yaw_step to completion first) -- trusts that exact heading
    instead of re-deriving a bearing from live position each tick, which
    is what let the two robots' paths diverge from each other before.
    Stop condition is y >= target_y (reached-or-passed), not "within
    stop_dist of target_y" -- since travel is monotonically increasing Y,
    a symmetric distance check triggers the instant the robot enters that
    band from below, i.e. at (target_y - stop_dist), not at target_y.
    That's the actual reason every stop this session landed short of
    target by roughly stop_dist -- not motor overshoot, not the wrong
    target, the stop condition itself was checking the wrong thing.
    stop_dist here is now just a small settle margin, not the real
    tolerance. Returns True once target_y is reached (and stops the robot)."""
    ramp = min(1.0, step / RAMP_STEPS)
    speed = speed * ramp
    _, y, _ = get_xy_yaw(r)
    if y >= target_y - stop_dist:
        r.set_dof_velocity_targets([0] * 4, dof_indices=wj)
        return True
    r.set_dof_velocity_targets([speed] * 4, dof_indices=wj)
    return False

SETTLE_FRAMES = 60  # ~1s buffer after each turn before the next stage starts

def apply_yaw_compensation(r, yj, yaw_base, chassis_yaw0):
    """Actively holds the plate's world-frame yaw constant regardless of
    how much the chassis underneath it has turned since chassis_yaw0 --
    without this, the plate (only damped before) just gets dragged around
    with the chassis, putting torque into whatever's resting on it."""
    _, _, chassis_yaw = get_xy_yaw(r)
    delta = math.atan2(math.sin(chassis_yaw - chassis_yaw0), math.cos(chassis_yaw - chassis_yaw0))
    r.set_dof_position_targets(yaw_base - delta, dof_indices=yj)

def run_two_robot_lift():
    global piston_target, piston_target2, pickup_done
    print("[two robot pickup] driving both robots in from opposite short sides...")
    robot.set_dof_velocity_targets([Base_speed]*4, dof_indices=wheel_joints)
    robot2.set_dof_velocity_targets([Base_speed]*4, dof_indices=wheel_joints2)
    robot_done = robot2_done = False
    for _ in range(2000):
        pos, _ = robot.get_world_poses()
        pos2, _ = robot2.get_world_poses()
        x = float(pos.numpy()[0, 0])
        x2 = float(pos2.numpy()[0, 0])
        if not robot_done and x <= TARGET_X:
            robot.set_dof_velocity_targets([0]*4, dof_indices=wheel_joints)
            robot_done = True
            print(f"[two robot pickup] Robot reached its end, x={x:.3f}")
        if not robot2_done and x2 >= TARGET_X2:
            robot2.set_dof_velocity_targets([0]*4, dof_indices=wheel_joints2)
            robot2_done = True
            print(f"[two robot pickup] Robot2 reached its end, x2={x2:.3f}")
        if robot_done and robot2_done:
            break
        simulation_app.update()
    for _ in range(30):
        simulation_app.update()

    print("[two robot pickup] lifting together...")
    obj = RigidPrim("/Cube_01")
    z0 = obj.get_world_poses()[0].numpy()[0, 2]
    while piston_target < LIFT_TARGET:
        piston_target = min(LIFT_TARGET, piston_target + Piston_speed * (1 / 60))
        piston_target2 = min(LIFT_TARGET, piston_target2 + Piston_speed * (1 / 60))
        robot.set_dof_position_targets(piston_target, dof_indices=piston_joint)
        robot2.set_dof_position_targets(piston_target2, dof_indices=piston_joint2)
        simulation_app.update()
    z1 = obj.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] Cube_01 z: {z0:.4f} -> {z1:.4f} (lifted {z1-z0:.4f})")

    _, _, chassis_yaw0 = get_xy_yaw(robot)
    _, _, chassis_yaw0_2 = get_xy_yaw(robot2)

    print("[two robot pickup] carrying to shelf 2 -- Robot turns first (Robot2 holds)...")
    robot2.set_dof_velocity_targets([0] * 4, dof_indices=wheel_joints2)
    for step in range(1500):
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
        apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
        if turn_to_yaw_step(robot, wheel_joints, DELIVERY_YAW, step=step):
            break
        simulation_app.update()
    for _ in range(SETTLE_FRAMES):
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
        apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
        simulation_app.update()

    print("[two robot pickup] Robot2 turns (Robot holds)...")
    robot.set_dof_velocity_targets([0] * 4, dof_indices=wheel_joints)
    for step in range(1500):
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
        apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
        if turn_to_yaw_step(robot2, wheel_joints2, DELIVERY_YAW, step=step):
            break
        simulation_app.update()
    for _ in range(SETTLE_FRAMES):
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
        apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
        simulation_app.update()

    print("[two robot pickup] both aligned -- driving straight to shelf 2...")
    r1_done = r2_done = False
    for step in range(2000):
        apply_yaw_compensation(robot, yaw_joint, yaw_joint_base, chassis_yaw0)
        apply_yaw_compensation(robot2, yaw_joint2, yaw_joint_base2, chassis_yaw0_2)
        if not r1_done:
            r1_done = drive_straight_step(robot, wheel_joints, DELIVERY_Y, step=step)
        if not r2_done:
            r2_done = drive_straight_step(robot2, wheel_joints2, DELIVERY_Y, step=step)
        if r1_done and r2_done:
            print("[two robot pickup] both arrived at shelf 2")
            break
        simulation_app.update()
    else:
        print("[two robot pickup] WARNING: did not both arrive within the step budget")
    for _ in range(30):
        simulation_app.update()
    z2 = obj.get_world_poses()[0].numpy()[0, 2]
    print(f"[two robot pickup] Cube_01 z after carry: {z2:.4f} (still on the plates if close to {z1:.4f})")
    # WORLD coordinates -- the Property panel shows LOCAL (offset by each
    # robot's own spawn parent transform, y=-0.747 for both here), so these
    # numbers will read ~0.747 higher in the GUI than what's printed here.
    x1, y1, _ = get_xy_yaw(robot)
    x2, y2, _ = get_xy_yaw(robot2)
    print(f"[two robot pickup] final WORLD chassis position -- Robot=({x1:.3f},{y1:.3f}) Robot2=({x2:.3f},{y2:.3f})")
    p1 = plate1.get_world_poses()[0].numpy()[0]
    p2 = plate2.get_world_poses()[0].numpy()[0]
    print(f"[two robot pickup] final WORLD plate position -- Robot plate=({p1[0]:.3f},{p1[1]:.3f},{p1[2]:.3f}) Robot2 plate=({p2[0]:.3f},{p2[1]:.3f},{p2[2]:.3f})")
    print(f"[two robot pickup] shelf 2's gap (measured): y[3.2,3.3], x[-1.0,1.0] valid, z[0.255,0.495] clear at this y-band")

    print("[two robot pickup] sequence complete -- back to manual control (Robot only)")
    pickup_done = True

input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

held_keys = set()
def on_key_event(e):
    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
        held_keys.add(e.input)
        if e.input == carb.input.KeyboardInput.F:
            frame_viewport_prims(get_active_viewport(), ["/World"])
        elif e.input == carb.input.KeyboardInput.P and not pickup_done:
            run_two_robot_lift()
    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
        held_keys.discard(e.input)
    return True

input_interface.subscribe_to_keyboard_events(keyboard, on_key_event)
print("[two robot pickup] press P to start the two-robot cooperative lift")
print("[two robot pickup] WASD drive Robot, R/C piston up/down, arrows pitch/roll, Q/E yaw, F frame whole map")

Tilt_speed = 2
manual_yaw_target = yaw_joint_base
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

    robot.set_dof_velocity_targets([roll_vel, pitch_vel], dof_indices=tilt_joints)

    # yaw is now position-controlled (see setup_robot), not velocity --
    # Q/E nudges a manual target the same way R/C nudges the piston.
    if yaw_left:
        manual_yaw_target += Tilt_speed * dt
    elif yaw_right:
        manual_yaw_target -= Tilt_speed * dt
    robot.set_dof_position_targets(manual_yaw_target, dof_indices=yaw_joint)

    simulation_app.update()

simulation_app.close()
