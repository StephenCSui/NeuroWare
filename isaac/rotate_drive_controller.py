#!/usr/bin/env python3
"""Rotate/drive-straight path executor for Robot 1.

Replaces Nav2's bt_navigator + controller_server (the RPP/DWB local
execution layer) while still using Nav2's planner_server for the actual
route -- planner_server hosts its own internal "global_costmap" (confirmed
this session via its own startup log lines, e.g. "[planner_server]
[global_costmap.global_costmap]: Activating"), fed live by the /scan
topic this robot already publishes, so routes still bend around real
obstacles. Only the LAST-MILE EXECUTION is custom here.

Runs as a plain rclpy node via system python3 -- NOT Isaac's python.sh /
Kit process, where rclpy is broken this session (PyUnicode_IS_READY
crash, see nav2_bridge_robot1.py's docstring). This node never touches
Isaac Sim directly; it only talks to the already-running bridge over
/odom, /scan, /cmd_vel, same as `ros2 topic pub` has been doing all
session.

Never blends rotation and translation -- every leg is either a pure
in-place turn or a pure straight drive, publishing angular.z XOR
linear.x, never both nonzero at once. Decided this session: this
project's environment is small-scale/tight-tolerance and will only get
tighter (more objects, less open space) later, so curvature-blended
correction (what Nav2's RPP does below its rotate_to_heading_min_angle)
is a liability, not an efficiency win, on a skid-steer chassis that
already tracks pure rotation and pure straight-line driving far more
predictably than any blended arc (established earlier this session:
DWB's arcing repeatedly failed to converge; RPP's blended phase was the
one behavior never fully characterized/trusted). Ramp/tolerance
constants below are ported directly from the same primitives already
proven in two_robot_pickup_demo.py (turn_to_yaw_step/drive_to_point_step),
just adapted from direct wheel-DOF control to /cmd_vel Twist messages
since this node has no direct Isaac Sim access.

Usage:
    python3 rotate_drive_controller.py
    # then, from anywhere:
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
        "{header: {frame_id: 'odom'}, pose: {position: {x: -2.532, y: 3.997}}}"
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

# Real max speeds, same as this robot's other scripts this session --
# Base_speed=5.0 wheel-rad/s * WHEEL_RADIUS=0.04 -> 0.2 m/s linear;
# 2*5.0*0.04/TRACK_WIDTH(0.174) -> ~2.30 rad/s pure-rotation max. Kept
# under those reals for margin, matching desired_linear_vel/
# rotate_to_heading_angular_vel already used in nav2_params_robot1.yaml.
LINEAR_SPEED = 0.3         # was 0.15 -- real physical max is ~0.2 m/s
                            # (Base_speed=5.0 wheel-rad/s * WHEEL_RADIUS),
                            # so this commands past the ceiling on
                            # purpose; the wheels just saturate there,
                            # no harm in asking for more than they can do.
ANGULAR_SPEED = 3.0        # was 1.0 -- real physical max is ~2.3 rad/s,
                            # same reasoning as LINEAR_SPEED above.
CARRY_ANGULAR_SPEED = ANGULAR_SPEED  # was a separate, slower 1.0 rad/s.
                            # That was solving the wrong problem: the
                            # plate swinging into things while carrying
                            # was traced (and fixed, in nav2_bridge_robot1.py)
                            # to a real bug -- a leftover manual-keyboard-
                            # control code path silently overwriting
                            # apply_yaw_compensation's target on the same
                            # joint every tick, unconditionally, so the
                            # compensation never reached the joint at all
                            # (not a lag issue). With that fixed, slowing
                            # carry turns down bought nothing and cost
                            # something real: at the chassis-to-wheel
                            # conversion, 1.0 rad/s chassis rate is only
                            # ~2.2 rad/s at the wheels, under half the
                            # 5.0 rad/s the original two_robot_pickup_demo.py
                            # actually used for its proven loaded turns --
                            # plausibly not enough torque/speed to reliably
                            # turn a loaded chassis, a real, different
                            # stall confirmed live this session (wheel
                            # velocities near-zero, no raycast hit nearby
                            # in any direction -- not a physical block).
RAMP_TIME = 1.5            # seconds to reach full speed from a standstill,
                            # matches RAMP_STEPS=90 @ ~60fps elsewhere
DECEL_DISTANCE = 0.3       # meters -- start slowing a drive leg within this
DECEL_ANGLE = math.radians(4)   # start slowing a turn within this -- was
                            # 15deg, live-tested this session and found
                            # that backing the commanded speed down for
                            # any turn under 15deg meant EVERY inter-
                            # waypoint turn (Nav2's grid path naturally
                            # has ~8-20deg headings between simplified
                            # waypoints) ran at a throttled speed this
                            # chassis can't reliably convert into real
                            # rotation (confirmed twice this session: a
                            # steady small command produces near-zero or
                            # sign-reversed actual motion; only a large,
                            # SUSTAINED command builds real momentum).
                            # 4deg means almost every turn runs at full
                            # ANGULAR_SPEED and only backs off right at
                            # the very end.
MIN_RAMP_OUT = 0.85        # floor on the decel ramp's speed multiplier --
                            # was 0.6 (=0.6 rad/s), inside the same
                            # measured dead zone. 0.85 (=0.85 rad/s) stays
                            # solidly above it even during the final
                            # deceleration tail. That dead-zone finding was
                            # measured for TURNS specifically (angular,
                            # rad/s) -- still used as-is for turns and for
                            # intermediate drive waypoints, where staying
                            # clear of it matters more than precision.
FINAL_APPROACH_MIN_RAMP_OUT = 0.3  # separate, lower floor used ONLY for
                            # the final approach to a leg's last waypoint
                            # (is_final) -- live-observed this session:
                            # at the shared 0.85 floor (=0.255 m/s), the
                            # chassis's own coast/momentum overshot the
                            # tight new GOAL_STOP_DIST (0.01m) at the
                            # shelf-2 staging point. This is a genuinely
                            # separate, untested value from the turn dead
                            # zone above (different axis, never measured
                            # for straight-line driving specifically) --
                            # needs live verification that it neither still
                            # overshoots nor introduces a new stall.
YAW_TOL = math.radians(1.5)     # aligned-enough to start driving straight
WAYPOINT_STOP_DIST = 0.05  # arrival tolerance for intermediate waypoints --
                            # left loose, these are just pass-through points
GOAL_STOP_DIST = 0.01      # arrival tolerance for the final goal -- was
                            # 0.05, tightened per direct instruction for the
                            # shelf-2 staging point specifically (needs to
                            # be very accurate before the heading correction
                            # runs), applied globally since every other
                            # final stop benefits from tighter arrival too.
                            # NOTE: MIN_RAMP_OUT below is a floor that keeps
                            # approach speed high right up to this
                            # threshold -- deliberately left untouched here
                            # (real risk of reopening an already-fixed wheel
                            # dead-zone problem); whether 0.01m is actually
                            # achievable at the current approach speed needs
                            # live verification, not assumed.
CONTROL_HZ = 20.0
OBSTACLE_STOP_RANGE = 0.25 # meters -- hard safety stop while not carrying,
                            # matches local_costmap's inflation_radius
FORWARD_HALF_ANGLE = math.radians(90.0)  # the reactive obstacle check only
                            # considers scan readings within this angle of
                            # straight ahead (body frame, so it rotates
                            # with the chassis -- during a TURN this is
                            # "ahead of wherever the nose currently points",
                            # sweeping toward the turn target). Was full
                            # 360 degrees -- live-caught this session: an
                            # obstacle already passed and now behind the
                            # robot (e.g. shelf 1's own corner leg, right
                            # after exiting) kept tripping the check purely
                            # because SOMETHING was close in ANY direction,
                            # with no notion of whether it was actually in
                            # the way. 90deg matches this scan's own
                            # per-camera FOV (SCAN_HORIZONTAL_FOV_DEG) --
                            # roughly the frontal hemisphere.
# Ported from the proven single-robot+obstacle test and
# two_robot_pickup_demo.py's current_rotation_radius/OBJECT_HALF_EXTENTS:
# that stack never uses an on/off suppression switch for carry legs, it
# uses ONE continuous check at all times with a threshold that grows to
# cover whatever's being carried. An on/off suppress flag (tried earlier
# this session) either blinded the check completely on carry legs (a real
# obstacle got driven straight through) or needed a guessed clearance
# floor to avoid self-triggering on the cargo's own presence -- both
# symptoms of using the wrong shape of fix. A PhysX-raycast identity
# filter was also tried and was a dead end for a different reason: the
# scene's obstacle prim has no physics collision at all (confirmed by the
# user), so a raycast can never see it regardless of tuning.
# CARGO_HALF_EXTENTS is /Cube's real half-extents, queried via the scene
# bbox dump this session (0.735,-0.922,...)-(0.936,-0.572,...) -- not
# guessed -- matching two_robot_pickup_demo.py's own OBJECT_HALF_EXTENTS
# entry for the same prim path almost exactly ((0.1001, 0.1750)).
CHASSIS_RADIUS = math.hypot(0.15, 0.075)          # real chassis half-extents
CARGO_HALF_EXTENTS = (0.1001, 0.1750)             # real /Cube half-extents
CARRY_OBSTACLE_STOP_RANGE = CHASSIS_RADIUS + math.hypot(*CARGO_HALF_EXTENTS)
STALL_TIMEOUT = 8.0        # seconds with no measurable yaw/position change
                            # during a turn/drive before aborting the leg

# An obstacle trip used to go straight to IDLE + replan from the exact
# spot that tripped it -- since the robot never physically moved, the
# fresh plan's first step landed in a position that was still inside
# `threshold`, so the very next control tick tripped the same check
# again before a single step of the new plan could execute (confirmed
# live: 6900+ stop/replan cycles, zero net progress, scan range flat at
# 0.31-0.36m the entire time). BACKOFF retraces a small fixed step, then
# lets the planner actually try again from there -- if that replanned
# leg trips the same check, it backs off another small step (cumulative)
# and retries, rather than either looping in place forever or trying to
# guess a single big retreat distance up front.
BACKOFF_SPEED = MIN_RAMP_OUT * LINEAR_SPEED  # reuse the already-tuned
                            # floor for reliable sustained motion instead
                            # of guessing a new low-speed constant
BACKOFF_STEP_DIST = 0.1    # meters retreated per attempt before
                            # requesting a fresh replan
BACKOFF_MAX_ATTEMPTS = 5   # consecutive backoff+replan attempts (without
                            # an intervening real waypoint reached) before
                            # giving up as a genuine stall instead of
                            # retrying forever -- caps cumulative retreat
                            # at BACKOFF_MAX_ATTEMPTS * BACKOFF_STEP_DIST
BACKOFF_TIMEOUT = 5.0      # seconds -- safety cap on a single backoff
                            # step if even that makes no odometry
                            # progress at all (e.g. boxed in from behind)

PATH_MIN_SEG = 0.15        # meters -- merge path points closer than this.
                            # Was 0.05 -- live-tested this session and
                            # found the grid-resolution path (NavfnPlanner,
                            # 0.1m costmap cells) kept far more waypoints
                            # than needed, making the robot stop to
                            # re-turn every ~0.1-0.2m instead of driving
                            # longer straight legs.
PATH_ANGLE_THRESH = math.radians(20.0)  # merge collinear-ish runs -- was
                            # 8deg, same reasoning: too sensitive to the
                            # planner's own grid stair-stepping, which
                            # isn't a REAL bend in the route, just
                            # discretization noise.

# Proactive replanning -- the planner previously only got asked again on a
# brand new goal or an already-tripped reactive stop, so the only way to
# find out a route had gone bad was to physically drive into it. This
# periodically checks the REMAINING path against the live global_costmap
# (already published on /global_costmap/costmap, nav_msgs/OccupancyGrid,
# confirmed live: publish_frequency=1.0, always_send_full_costmap=True in
# nav2_params_robot1.yaml) and requests a fresh plan the moment it's
# actually blocked -- while there's still room to route around it, instead
# of waiting for the reactive hard-stop.
LOOKAHEAD_SAMPLE_SPACING = PATH_MIN_SEG  # reuse the same 0.15m scale
                            # already used to simplify the planner's path
LOOKAHEAD_LETHAL_COST = 99  # nav2_costmap_2d's OccupancyGrid wire encoding
                            # (confirmed via ros2 topic info/echo against the
                            # live node): LETHAL_OBSTACLE -> 100,
                            # INSCRIBED_INFLATED_OBSTACLE (inside
                            # robot_radius of a real obstacle) -> 99,
                            # everything else is the softer inflation-cost
                            # gradient scaled below that. >=99 means
                            # "genuinely too close to be viable", not just
                            # "somewhat discouraged" -- reacting to any
                            # nonzero inflation cost would fire constantly
                            # in a tight scene and defeat the point of
                            # keeping this separate from the reactive check.
LOOKAHEAD_DEBOUNCE_COUNT = 2  # consecutive costmap messages (~2s at 1Hz)
                            # a block must persist across before triggering
                            # a proactive replan -- filters a single noisy/
                            # transient reading, same discipline as
                            # CAMERA_OVERLAP_RECONCILE_TOL/_median_smooth in
                            # nav2_bridge_robot1.py.


def simplify_path(points):
    """Collapse a dense grid-resolution path into turn points only, so the
    robot isn't stopping to re-align every 0.1m grid cell -- same idea as
    the occupancy_grid.py path-simplification planned earlier this
    session, applied here to Nav2's own NavfnPlanner output instead."""
    if len(points) <= 2:
        return points
    simplified = [points[0]]
    ref_dir = None
    for x, y in points[1:]:
        lx, ly = simplified[-1]
        dx, dy = x - lx, y - ly
        if math.hypot(dx, dy) < PATH_MIN_SEG:
            continue
        this_dir = math.atan2(dy, dx)
        if ref_dir is None or abs(math.atan2(math.sin(this_dir - ref_dir), math.cos(this_dir - ref_dir))) > PATH_ANGLE_THRESH:
            simplified.append((x, y))
            ref_dir = this_dir
        else:
            simplified[-1] = (x, y)
    if simplified[-1] != points[-1]:
        simplified.append(points[-1])
    return simplified


class RotateDriveController(Node):
    def __init__(self):
        super().__init__("rotate_drive_controller")

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.have_odom = False
        self.min_scan_range = float("inf")

        self.goal_xy = None
        self.waypoints = []
        self.state = "IDLE"  # IDLE | TURN | DRIVE | BACKOFF
        self.leg_start_time = None
        self.leg_start_yaw = None
        self.leg_start_pos = None
        self.last_progress_time = None
        self.backoff_attempts = 0
        self.backoff_start_pos = None
        self.backoff_start_time = None
        self._replan_reason = "initial plan"
        self._waypoint_total = 0
        self._waypoint_done = 0
        self._latest_costmap = None
        self._costmap_block_streak = 0
        self._last_synced_radius = None
        self.angular_speed = ANGULAR_SPEED  # overridden to CARRY_ANGULAR_SPEED
                                            # by /goal_pose_carry_direct

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        # /goal_pose_approach: still planner-routed (real detour-finding
        # around obstacles kept), not a carry leg -- obstacle-stop check
        # uses the empty-handed threshold, same as /goal_pose.
        self.create_subscription(PoseStamped, "/goal_pose_approach", self._goal_approach_cb, 10)
        # /goal_pose_carry_planned: the same planner_server-routed pathway
        # as /goal_pose, carrying-aware (see _goal_carry_planned_cb).
        self.create_subscription(PoseStamped, "/goal_pose_carry_planned", self._goal_carry_planned_cb, 10)
        # /goal_pose_direct: no planner routing at all, no obstacle check
        # -- a single turn-to-exact-bearing-then-drive-straight leg,
        # matching two_robot_pickup_demo.py's original run_goto exactly.
        # For the final pickup approach specifically: confirmed live this
        # session that a planner-routed approach can arrive at the
        # object's (x,y) from an arbitrary final heading (whatever the
        # last routed waypoint's bearing happens to be), which doesn't
        # reliably match the shelf's real clearance angle -- the plate
        # physically hit the shelf's rail structure on approach. The
        # original script always turns to the EXACT bearing from current
        # position straight to the object and drives straight in, so the
        # final approach angle is consistent every time.
        self.create_subscription(PoseStamped, "/goal_pose_direct", self._goal_direct_cb, 10)
        # /goal_pose_carry_direct: same as /goal_pose_direct (no planner
        # routing, no obstacle check), but turns use CARRY_ANGULAR_SPEED
        # instead of the full ANGULAR_SPEED -- for turns made while
        # carrying cargo, where the plate's counter-rotation (a real
        # position-target joint, not instantaneous) needs to keep pace
        # with the chassis or the carried object swings into things.
        self.create_subscription(PoseStamped, "/goal_pose_carry_direct", self._goal_carry_direct_cb, 10)
        # /goal_pose_carry_clear_direct: same as /goal_pose_carry_direct,
        # but obstacle check off -- for the short exit-clearance hop
        # immediately after lifting, whose start point is right next to the
        # shelf just picked from by design (confirmed live this session:
        # the real /scan reading right after lift is ~0.17-0.36m, the shelf
        # structure itself, not a real obstacle -- the actual known
        # obstacle, Cube_03, is well past this leg's target, only relevant
        # on the longer carry-direct leg that follows).
        self.create_subscription(PoseStamped, "/goal_pose_carry_clear_direct", self._goal_carry_clear_direct_cb, 10)
        # /goal_pose_correct_heading: pure in-place rotation to an exact
        # absolute heading (msg.pose.orientation's yaw), position ignored
        # -- current position is used as the "target" so the DRIVE phase
        # completes trivially (remaining distance ~0) right after the
        # TURN converges. Used for the shelf-2 staging-point correction:
        # bring the chassis back to the exact heading it had leaving
        # shelf 1, independent of wherever navigation left it facing.
        self.create_subscription(PoseStamped, "/goal_pose_correct_heading", self._goal_correct_heading_cb, 10)
        # Real carrying state, not an on/off suppress flag -- drives which
        # obstacle-stop threshold _control_tick uses (CARRY_OBSTACLE_STOP_RANGE
        # vs OBSTACLE_STOP_RANGE), same as two_robot_pickup_demo.py's
        # current_rotation_radius(label) picking its radius from the real
        # `carrying` dict rather than a per-leg-type switch.
        self.carrying = False
        self.direct_mode = False  # True for /goal_pose_direct and
                                   # /goal_pose_carry_direct -- no planner
                                   # routing, so an obstacle stop can't
                                   # trigger a replan on these legs, just stop
        # /goal_pose_direct's target IS the object about to be picked up --
        # the obstacle check must be off for that leg specifically (not
        # just "less strict"), confirmed live this session: with it on,
        # the robot stalled 0.24m short of the pickup point forever (no
        # replan possible on a direct leg) because its own destination
        # reads as "an obstacle ahead." /goal_pose_carry_direct's target is
        # empty floor / shelf2, where anything in between (Cube_03) really
        # is an obstacle, so that leg keeps the check. Set per-callback,
        # not derived from direct_mode/carrying.
        self.obstacle_check_enabled = True

        # transient_local so a subscriber that starts AFTER this node
        # (e.g. an orchestration script launched later) still gets the
        # current status immediately instead of missing it.
        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/nav_status", status_qos)
        self._status = None
        self._publish_status("idle")

        self._plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        # Real node/topic/service names confirmed live against the running
        # planner_server (ros2 node/topic/service list), not assumed --
        # the costmap runs as its own node ("/global_costmap/global_costmap")
        # hosted inside the planner_server process, matching the
        # nav2_params_robot1.yaml global_costmap/global_costmap nesting.
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, 10)
        self._radius_param_client = self.create_client(
            SetParameters, "/global_costmap/global_costmap/set_parameters")

        self.create_timer(1.0 / CONTROL_HZ, self._control_tick)
        self.get_logger().info("rotate_drive_controller ready -- publish a PoseStamped to /goal_pose to start")

    def _publish_status(self, status):
        if status != self._status:
            self._status = status
            self.status_pub.publish(String(data=status))

    # -- callbacks --------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x, self.cur_y = p.x, p.y
        self.cur_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def _scan_cb(self, msg: LaserScan):
        angle = msg.angle_min
        forward = []
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0 and abs(angle) <= FORWARD_HALF_ANGLE:
                forward.append(r)
            angle += msg.angle_increment
        self.min_scan_range = min(forward) if forward else float("inf")

    def _costmap_cb(self, msg: OccupancyGrid):
        self._latest_costmap = msg
        # direct_mode legs are never planner-routed, by design (a routed
        # approach was confirmed to clip shelf rail structure) -- the
        # reactive check already special-cases this (stops but never
        # replans a direct leg); the proactive check must respect the same
        # rule or it can silently overwrite a direct leg's single-waypoint
        # route with a multi-waypoint planner route.
        if (self.state in ("TURN", "DRIVE") and self.waypoints
                and self.obstacle_check_enabled and not self.direct_mode):
            self._lookahead_check()

    def _sample_remaining_path(self):
        """Points along current-position -> each remaining waypoint, at
        LOOKAHEAD_SAMPLE_SPACING intervals -- the same scale simplify_path
        already collapses the planner's own output to."""
        pts = []
        px, py = self.cur_x, self.cur_y
        for wx, wy in self.waypoints:
            seg_len = math.hypot(wx - px, wy - py)
            steps = max(1, int(seg_len / LOOKAHEAD_SAMPLE_SPACING))
            for s in range(1, steps + 1):
                t = min(1.0, s / steps)
                pts.append((px + (wx - px) * t, py + (wy - py) * t))
            px, py = wx, wy
        return pts

    def _lookahead_check(self):
        """Proactive replan trigger: samples the REMAINING path against the
        live global_costmap and requests a fresh plan the moment it's
        actually blocked, debounced across LOOKAHEAD_DEBOUNCE_COUNT
        messages so a single noisy reading doesn't cause a replan. This is
        separate from _control_tick's reactive obstacle_present check --
        that one is the last-resort hard stop for a genuine surprise; this
        one is meant to make that hard stop rare by catching a bad route
        while there's still room to fix it, using the same live costmap the
        planner itself already searches over (assumes the rolling-window
        costmap's origin is axis-aligned, i.e. no yaw -- true for Nav2's
        standard rolling-window costmap implementation)."""
        grid = self._latest_costmap
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        w, h = grid.info.width, grid.info.height
        blocked = False
        for x, y in self._sample_remaining_path():
            col = int((x - ox) / res)
            row = int((y - oy) / res)
            if 0 <= col < w and 0 <= row < h:
                if grid.data[row * w + col] >= LOOKAHEAD_LETHAL_COST:
                    blocked = True
                    break
        self._costmap_block_streak = self._costmap_block_streak + 1 if blocked else 0
        if self._costmap_block_streak >= LOOKAHEAD_DEBOUNCE_COUNT:
            self._costmap_block_streak = 0
            self.get_logger().warn("proactive replan: path now blocked ahead (live costmap)")
            self._replan_reason = "proactive: path blocked in live costmap"
            if self.goal_xy:
                self._request_plan(self.goal_xy)

    def _goal_cb(self, msg: PoseStamped):
        self.carrying = False
        self.direct_mode = False
        self.obstacle_check_enabled = True
        self.angular_speed = ANGULAR_SPEED
        self._handle_goal(msg)

    def _goal_approach_cb(self, msg: PoseStamped):
        self.carrying = False
        self.direct_mode = False
        self.obstacle_check_enabled = True
        self.angular_speed = ANGULAR_SPEED
        self._handle_goal(msg)

    def _goal_carry_planned_cb(self, msg: PoseStamped):
        # Same real planner_server-routed path as /goal_pose (real
        # ComputePathToPose routing, real obstacle-triggered replan via
        # _control_tick's non-direct-mode branch) -- this exact pathway
        # was built and live-verified for obstacle avoidance against the
        # single-robot+Cube_03 world before being ported to the full
        # two-shelf world, per the project log. It was only ever validated
        # empty-handed, though: this callback is the carrying-aware
        # wrapper around the identical machinery -- CARRY_ANGULAR_SPEED so
        # the plate's counter-rotation can keep pace with routed turns,
        # and carrying=True so _control_tick uses CARRY_OBSTACLE_STOP_RANGE
        # instead of the empty-handed threshold.
        self.carrying = True
        self.direct_mode = False
        self.obstacle_check_enabled = True
        self.angular_speed = CARRY_ANGULAR_SPEED
        self._handle_goal(msg)

    def _goal_direct_cb(self, msg: PoseStamped):
        self.carrying = False
        self.direct_mode = True
        # Off, not just relaxed -- this leg's target IS the pickup object.
        self.obstacle_check_enabled = False
        self.angular_speed = ANGULAR_SPEED
        self._handle_direct_goal(msg, label="direct, no planning")

    def _goal_carry_direct_cb(self, msg: PoseStamped):
        self.carrying = True
        self.direct_mode = True
        self.obstacle_check_enabled = True
        self.angular_speed = CARRY_ANGULAR_SPEED
        self._handle_direct_goal(msg, label="carry-direct, no planning, slow turn")

    def _goal_carry_clear_direct_cb(self, msg: PoseStamped):
        self.carrying = True
        self.direct_mode = True
        self.obstacle_check_enabled = False
        self.angular_speed = CARRY_ANGULAR_SPEED
        self._handle_direct_goal(msg, label="carry-clear-direct, no planning, slow turn, obstacle check off")

    def _goal_correct_heading_cb(self, msg: PoseStamped):
        self.carrying = True
        self.direct_mode = True
        self.obstacle_check_enabled = True
        self.angular_speed = CARRY_ANGULAR_SPEED
        q = msg.pose.orientation
        target_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.goal_xy = (self.cur_x, self.cur_y)
        self.get_logger().info(f"new goal received (correct-heading, in-place, target={math.degrees(target_yaw):.1f}deg): {self.goal_xy}")
        self._publish_status("driving")
        self.waypoints = [self.goal_xy]
        self._waypoint_done = 0
        self._waypoint_total = 1
        self.state = "TURN"
        self._start_leg(absolute_yaw=target_yaw)

    def _handle_direct_goal(self, msg: PoseStamped, label):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"new goal received ({label}): {self.goal_xy}")
        self._publish_status("driving")
        self.waypoints = [self.goal_xy]
        self._waypoint_done = 0
        self._waypoint_total = 1
        self.state = "TURN"
        self._start_leg(turn_target=self.waypoints[0])

    def _handle_goal(self, msg: PoseStamped):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"new goal received: {self.goal_xy}")
        self.backoff_attempts = 0
        self._publish_status("driving")
        self._replan_reason = "new goal"
        self._request_plan(self.goal_xy)

    # -- planning -----------------------------------------------------
    def _required_clearance(self):
        """The same threshold _control_tick's reactive check uses -- kept
        as one function so the planner and the reactive check can never
        drift apart onto two different numbers for the same state."""
        return CARRY_OBSTACLE_STOP_RANGE if self.carrying else OBSTACLE_STOP_RANGE

    def _request_plan(self, goal_xy):
        if not self._plan_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("planner_server action not available yet")
            return
        required_radius = self._required_clearance()
        # Sync the planner's robot_radius to the real current requirement
        # before asking it to plan -- it defaulted to a static 0.20
        # regardless of carrying state, so it was routing against a
        # smaller boundary than what _control_tick's reactive check
        # actually enforces, producing routes that looked valid to the
        # planner but always got rejected on execution. Only calls the
        # param service when the value actually needs to change (tracked
        # via _last_synced_radius), not on every single leg.
        if required_radius == self._last_synced_radius or not self._radius_param_client.service_is_ready():
            if required_radius != self._last_synced_radius:
                self.get_logger().warn(
                    "global_costmap set_parameters service not ready -- planning with stale robot_radius")
            self._send_plan_goal(goal_xy)
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="robot_radius",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=required_radius),
        )]
        future = self._radius_param_client.call_async(req)
        def _on_radius_synced(fut, radius=required_radius, goal=goal_xy):
            self._last_synced_radius = radius
            self._send_plan_goal(goal)
        future.add_done_callback(_on_radius_synced)

    def _send_plan_goal(self, goal_xy):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = "odom"
        goal_msg.goal.pose.position.x = goal_xy[0]
        goal_msg.goal.pose.position.y = goal_xy[1]
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = "GridBased"
        future = self._plan_client.send_goal_async(goal_msg)
        future.add_done_callback(self._plan_goal_response)

    def _plan_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("planner rejected the goal")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result)

    def _plan_result(self, future):
        path: Path = future.result().result.path
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        if not points:
            self.get_logger().warn("planner returned an empty path")
            return
        new_waypoints = simplify_path(points)[1:]  # drop the start pose
        if not new_waypoints:
            return
        # A replan shouldn't interrupt a leg already in progress toward
        # essentially the same target -- doing so was restarting the
        # ramp-up from zero, measured live this session as choppy speed
        # and much slower net progress than the commanded max. Only reset
        # the ramp if the immediate target actually changed. The reactive
        # obstacle trigger clears state to IDLE first so it never hits this
        # path, but the proactive _lookahead_check trigger deliberately
        # DOES hit it -- it leaves state/waypoints untouched when
        # requesting a replan, specifically so a route that turns out to be
        # still fine just continues without a ramp restart, and only a
        # genuinely different route below causes one.
        if self.waypoints and self.state in ("TURN", "DRIVE"):
            cx, cy = self.waypoints[0]
            nx, ny = new_waypoints[0]
            if math.hypot(cx - nx, cy - ny) < 0.1:
                old_total = self._waypoint_total
                self.waypoints = [(cx, cy)] + new_waypoints[1:]
                new_total = self._waypoint_done + len(self.waypoints)
                if new_total != old_total:
                    self.get_logger().info(
                        f"replanned ({self._replan_reason}): route length {old_total} -> {new_total} "
                        f"waypoint(s), current leg unchanged")
                    self._waypoint_total = new_total
                return
        old_total = self._waypoint_total
        self.waypoints = new_waypoints
        self._waypoint_done = 0
        self._waypoint_total = len(self.waypoints)
        change = f", was {old_total}" if old_total and old_total != self._waypoint_total else ""
        self.get_logger().info(
            f"path planned ({self._replan_reason}): {self._waypoint_total} waypoint(s){change}")
        self._publish_status("driving")
        self.state = "TURN"
        self._start_leg(turn_target=self.waypoints[0])

    # -- leg state machine --------------------------------------------
    def _start_leg(self, turn_target=None, absolute_yaw=None):
        self.leg_start_time = time.monotonic()
        self.leg_start_yaw = self.cur_yaw
        self.leg_start_pos = (self.cur_x, self.cur_y)
        self.last_progress_time = time.monotonic()
        # Bearing locked in ONCE here, not recomputed every tick in
        # _turn_step -- matches turn_to_yaw_step's proven design in
        # two_robot_pickup_demo.py. Recomputing bearing_to() every tick
        # from the live (slightly noisy) current position was measured
        # this session to make the commanded speed swing wildly (e.g.
        # 0.6 -> 0.2 -> 1.0 rad/s from tick to tick) whenever the target
        # waypoint is close to the robot, since a tiny position wobble
        # swings the bearing a lot at short range.
        if absolute_yaw is not None:
            # A pure in-place rotation to a specific heading, independent
            # of any target point -- e.g. the shelf-2 staging-point
            # correction, which needs the chassis back at the exact
            # heading it had leaving shelf 1, not a bearing to anywhere.
            self.leg_target_yaw = absolute_yaw
        elif turn_target is not None:
            tx, ty = turn_target
            self.leg_target_yaw = math.atan2(ty - self.cur_y, tx - self.cur_x)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _control_tick(self):
        if not self.have_odom or self.state == "IDLE" or not self.waypoints:
            return

        # Threshold grows to cover whatever's being carried instead of an
        # on/off switch based on carrying -- see CARRY_OBSTACLE_STOP_RANGE's
        # comment above. obstacle_check_enabled is a separate, per-leg-type
        # switch: off only for the direct pickup-approach leg, whose target
        # is intentionally the object about to be touched.
        threshold = self._required_clearance()
        obstacle_present = (self.state != "BACKOFF" and self.obstacle_check_enabled
                             and self.min_scan_range < threshold)
        if obstacle_present:
            self._stop()
            self._publish_status("stalled")
            if self.direct_mode:
                # Direct/carry-direct legs are intentionally not
                # planner-routed (a planner-routed approach was confirmed
                # this session to clip shelf rail structure on final
                # approach) -- so unlike the non-direct case, this doesn't
                # request a replan, it just stops. The orchestrator layer
                # decides what to do about a real obstacle appearing
                # mid-leg.
                self.get_logger().warn(f"obstacle at {self.min_scan_range:.2f}m on direct leg "
                                        f"(carrying={self.carrying}, threshold={threshold:.2f}m) -- stopping")
                self.waypoints = []
                self.state = "IDLE"
            elif self.backoff_attempts >= BACKOFF_MAX_ATTEMPTS:
                self.get_logger().error(
                    f"obstacle at {self.min_scan_range:.2f}m -- gave up after "
                    f"{BACKOFF_MAX_ATTEMPTS} backoff+replan attempts "
                    f"({BACKOFF_MAX_ATTEMPTS * BACKOFF_STEP_DIST:.2f}m retraced), no clear path found")
                self.waypoints = []
                self.state = "IDLE"
            else:
                self.backoff_attempts += 1
                self.get_logger().warn(
                    f"obstacle at {self.min_scan_range:.2f}m -- backing off "
                    f"{BACKOFF_STEP_DIST}m (attempt {self.backoff_attempts}/{BACKOFF_MAX_ATTEMPTS}) before replanning")
                self.backoff_start_pos = (self.cur_x, self.cur_y)
                self.backoff_start_time = time.monotonic()
                self.state = "BACKOFF"
            return

        if self.state == "BACKOFF":
            done = self._backoff_step()
            if done:
                self.waypoints = []
                self.state = "IDLE"
                self._replan_reason = f"post-backoff replan (attempt {self.backoff_attempts}/{BACKOFF_MAX_ATTEMPTS})"
                if self.goal_xy:
                    self._request_plan(self.goal_xy)
            return

        target_x, target_y = self.waypoints[0]
        is_final = len(self.waypoints) == 1
        stop_dist = GOAL_STOP_DIST if is_final else WAYPOINT_STOP_DIST

        if self.state == "TURN":
            done = self._turn_step(target_x, target_y)
            if done:
                self.state = "DRIVE"
                self._start_leg()
        elif self.state == "DRIVE":
            done = self._drive_step(target_x, target_y, stop_dist, is_final)
            if done:
                self.waypoints.pop(0)
                self._waypoint_done += 1
                if not self.waypoints:
                    self._stop()
                    self.get_logger().info(f"goal reached (waypoint {self._waypoint_done}/{self._waypoint_total})")
                    self._publish_status("reached")
                    self.state = "IDLE"
                    self.backoff_attempts = 0
                else:
                    self.get_logger().info(f"waypoint {self._waypoint_done}/{self._waypoint_total} reached")
                    self.state = "TURN"
                    self.backoff_attempts = 0
                    self._start_leg(turn_target=self.waypoints[0])

    def _turn_step(self, target_x, target_y):
        yaw_err = math.atan2(math.sin(self.leg_target_yaw - self.cur_yaw), math.cos(self.leg_target_yaw - self.cur_yaw))
        if abs(yaw_err) <= YAW_TOL:
            self._stop()
            return True
        if abs(math.atan2(math.sin(self.cur_yaw - self.leg_start_yaw), math.cos(self.cur_yaw - self.leg_start_yaw))) > math.radians(2.0):
            self.last_progress_time = time.monotonic()
            self.leg_start_yaw = self.cur_yaw
        elif time.monotonic() - self.last_progress_time > STALL_TIMEOUT:
            self.get_logger().warn("turn stalled -- aborting leg")
            self._stop()
            self.waypoints.pop(0)
            self.state = "TURN" if self.waypoints else "IDLE"
            if self.waypoints:
                self._start_leg(turn_target=self.waypoints[0])
            return False
        elapsed = time.monotonic() - self.leg_start_time
        ramp_in = min(1.0, elapsed / RAMP_TIME)
        ramp_out = max(MIN_RAMP_OUT, min(1.0, abs(yaw_err) / DECEL_ANGLE))
        speed = self.angular_speed * min(ramp_in, ramp_out)
        cmd = Twist()
        cmd.angular.z = speed if yaw_err > 0 else -speed
        self.cmd_pub.publish(cmd)
        return False

    def _drive_step(self, target_x, target_y, stop_dist, is_final=False):
        remaining = (target_x - self.cur_x) * math.cos(self.cur_yaw) + (target_y - self.cur_y) * math.sin(self.cur_yaw)
        if remaining <= stop_dist:
            self._stop()
            return True
        moved = math.hypot(self.cur_x - self.leg_start_pos[0], self.cur_y - self.leg_start_pos[1])
        if moved > 0.02:
            self.last_progress_time = time.monotonic()
            self.leg_start_pos = (self.cur_x, self.cur_y)
        elif time.monotonic() - self.last_progress_time > STALL_TIMEOUT:
            self.get_logger().warn("drive stalled -- aborting leg")
            self._stop()
            self.waypoints.pop(0)
            self.state = "TURN" if self.waypoints else "IDLE"
            if self.waypoints:
                self._start_leg(turn_target=self.waypoints[0])
            return False
        elapsed = time.monotonic() - self.leg_start_time
        ramp_in = min(1.0, elapsed / RAMP_TIME)
        floor = FINAL_APPROACH_MIN_RAMP_OUT if is_final else MIN_RAMP_OUT
        ramp_out = max(floor, min(1.0, remaining / DECEL_DISTANCE))
        speed = LINEAR_SPEED * min(ramp_in, ramp_out)
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)
        return False

    def _backoff_step(self):
        moved = math.hypot(self.cur_x - self.backoff_start_pos[0], self.cur_y - self.backoff_start_pos[1])
        if moved >= BACKOFF_STEP_DIST:
            self._stop()
            return True
        if time.monotonic() - self.backoff_start_time > BACKOFF_TIMEOUT:
            self.get_logger().warn(f"backoff made no odometry progress within {BACKOFF_TIMEOUT}s -- replanning anyway")
            self._stop()
            return True
        cmd = Twist()
        cmd.linear.x = -BACKOFF_SPEED
        self.cmd_pub.publish(cmd)
        return False


def main():
    rclpy.init()
    node = RotateDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
