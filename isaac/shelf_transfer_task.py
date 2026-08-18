#!/usr/bin/env python3
"""Orchestrates a single shelf-1 -> shelf-2 object transfer using this
session's new navigation stack (rotate_drive_controller.py + standalone
planner_server) plus nav2_bridge_robot1.py's remote piston control.

Sequence (real positions queried live, never hardcoded): drive directly
to the object -> lift -> verify it's actually on the plate (not just that
the lift command completed) -> rotate to face shelf 2 and drive a short
clearance distance, as its own explicit, verified step -> continue
straight to shelf 2 -> verify still attached -> lower.

This script only talks to already-running processes (the bridge, via its
polled /tmp control files; rotate_drive_controller.py, via ROS2 topics) --
it doesn't touch Isaac Sim directly, same as everything else built this
session that isn't inside the Kit process.

Usage:
    python3 shelf_transfer_task.py [object_prim] [shelf2_reference_prim]
    # defaults: /Cube, /World/Shelf2
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

QUERY_MARKER_FILE = "/tmp/robot1_query_marker"
QUERY_RESPONSE_FILE = "/tmp/robot1_query_response"
PISTON_CONTROL_FILE = "/tmp/robot1_piston_cmd"
PISTON_STATUS_FILE = "/tmp/robot1_piston_status"
CARGO_STATUS_FILE = "/tmp/robot1_cargo_status"
PLATE_ORIENTATION_STATUS_FILE = "/tmp/robot1_plate_orientation"
YAW_COMPENSATION_CONTROL_FILE = "/tmp/robot1_yaw_compensation"
DIAG_FILE = "/tmp/robot1_diag"
DIAG_RESPONSE_FILE = "/tmp/robot1_diag_response"
SCENE_DUMP_FILE = "/tmp/robot1_scene_dump"
SCENE_DUMP_RESPONSE_FILE = "/tmp/robot1_scene_dump_response"
# Debug-vis: this node has no direct USD access either (same constraint
# as rotate_drive_controller.py), so the exit/staging/entry points this
# script computes are handed over via a polled control file. One
# "label,odom_x,odom_y" per line, rewritten in full each time a new
# point is known -- the bridge converts to world and draws markers.
DEBUG_MARKERS_FILE = "/tmp/robot1_debug_markers"

QUERY_TIMEOUT = 5.0
STATUS_TIMEOUT = 300.0  # generous -- this session's sim has run well
                        # under real-time at times; a live-watched task
                        # shouldn't get cut short by an impatient timeout
EXIT_CLEARANCE_DISTANCE = 1.45  # meters to drive, after rotating to face
                        # shelf 2, before treating shelf 1 as cleared. Was
                        # 1.0, sized against the bare chassis radius
                        # (~0.168m) -- wrong reference: the NEXT leg
                        # (carry_planned) re-enables obstacle checking
                        # using CARRY_OBSTACLE_STOP_RANGE (~0.3696m, chassis
                        # + carried cargo), not the bare chassis radius. At
                        # 1.0m the exit point landed right beside shelf 1's
                        # own corner leg (/World/Shelf/Cube_04, confirmed via
                        # headless query at real clearance 0.124m), so the
                        # very next leg's obstacle check immediately
                        # re-tripped on shelf 1's own structure before ever
                        # reaching open corridor. 1.45m gives ~0.42m real
                        # clearance from that same corner leg, past
                        # CARRY_OBSTACLE_STOP_RANGE with margin. Geometry
                        # unaffected by the shelf-2/obstacle relayout (shelf
                        # 1 / Cube_04 were never moved). NOTE: this is
                        # chassis-only clearance -- the carried cargo's own
                        # footprint through this same gap during the blind
                        # (obstacle-check-off) exit leg has not been
                        # independently verified.
ENTRY_CLEARANCE_DISTANCE = 1.0  # meters short of shelf 2's real target --
                        # same margin as EXIT_CLEARANCE_DISTANCE, mirrored
                        # on the shelf 2 side: a staging point to arrive at
                        # with obstacle checking still on, before the final
                        # obstacle-check-off entry leg into the shelf
SETTLE_VELOCITY_TOL = 0.05  # rad/s -- real wheel speed under this counts
                        # as stopped, not just commanded-to-stop
SETTLE_TIMEOUT = 5.0   # generous -- the controller's own ramp should
                        # already have this near zero by the time "reached"
                        # fires; this is confirming it, not waiting out a
                        # long coast
STEP_SETTLE_PAUSE = 1.0  # seconds -- fixed pause after each key step is
                        # confirmed complete before starting the next one,
                        # per direct instruction: don't chain the next
                        # action immediately off a "done" status, give
                        # everything a beat first. Extra margin on top of
                        # the real physical-settle checks, not instead of
                        # them.


def query_prim_odom_xy(prim_path):
    """Reads a prim's live world position back converted to odom frame,
    via nav2_bridge_robot1.py's existing remote-query mechanism -- not
    guessed/hardcoded, the real current position."""
    try:
        import os
        if os.path.exists(QUERY_RESPONSE_FILE):
            os.remove(QUERY_RESPONSE_FILE)
    except OSError:
        pass
    with open(QUERY_MARKER_FILE, "w") as f:
        f.write(prim_path)
    deadline = time.monotonic() + QUERY_TIMEOUT
    while time.monotonic() < deadline:
        try:
            with open(QUERY_RESPONSE_FILE) as f:
                content = f.read().strip()
            if content:
                # nav2_bridge_robot1.py's response now has a 3rd (Z) field
                # -- unpack only the first two so this existing caller
                # (X/Y only) is unaffected by that addition.
                x, y, *_ = (float(v) for v in content.split(","))
                return x, y
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise TimeoutError(f"no query response for {prim_path} within {QUERY_TIMEOUT}s -- is nav2_bridge_robot1.py running and playing?")


def query_prim_odom_xyz(prim_path):
    """Same as query_prim_odom_xy but also returns world Z (unaffected by
    the odom yaw rotation, so it's not itself an 'odom Z' -- there's no
    such thing here, just the real world height)."""
    try:
        import os
        if os.path.exists(QUERY_RESPONSE_FILE):
            os.remove(QUERY_RESPONSE_FILE)
    except OSError:
        pass
    with open(QUERY_MARKER_FILE, "w") as f:
        f.write(prim_path)
    deadline = time.monotonic() + QUERY_TIMEOUT
    while time.monotonic() < deadline:
        try:
            with open(QUERY_RESPONSE_FILE) as f:
                content = f.read().strip()
            if content:
                x, y, z = (float(v) for v in content.split(","))
                return x, y, z
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise TimeoutError(f"no query response for {prim_path} within {QUERY_TIMEOUT}s -- is nav2_bridge_robot1.py running and playing?")


def query_prim_odom_bbox_y(prim_path):
    """Real physical footprint (odom-frame Y range), not the prim's Xform
    origin -- a live-observed bug this session found the Xform origin sits
    well outside a shelf's actual geometry, so a target derived from it
    can land past the shelf entirely. Returns (y_min, y_max)."""
    import os
    import re
    if os.path.exists(SCENE_DUMP_RESPONSE_FILE):
        os.remove(SCENE_DUMP_RESPONSE_FILE)
    with open(SCENE_DUMP_FILE, "w") as f:
        f.write(f"dump-{time.monotonic()}")
    deadline = time.monotonic() + QUERY_TIMEOUT
    while time.monotonic() < deadline:
        try:
            with open(SCENE_DUMP_RESPONSE_FILE) as f:
                content = f.read()
            for line in content.splitlines():
                if line.startswith(prim_path + " "):
                    m = re.search(r"odom_bbox=\(([-\d.]+),([-\d.]+)\)-\(([-\d.]+),([-\d.]+)\)", line)
                    if m:
                        y0, y1 = float(m.group(2)), float(m.group(4))
                        return min(y0, y1), max(y0, y1)
        except OSError:
            pass
        time.sleep(0.1)
    raise TimeoutError(f"no scene dump response for {prim_path} within {QUERY_TIMEOUT}s")


class ShelfTransferTask(Node):
    def __init__(self, object_prim, shelf2_prim):
        super().__init__("shelf_transfer_task")
        self.object_prim = object_prim
        self.shelf2_prim = shelf2_prim
        self.nav_status = None
        self.chassis_yaw = 0.0
        self.have_chassis_yaw = False
        self._debug_markers = {}  # debug-vis: label -> (odom_x, odom_y), accumulated over the run

        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/nav_status", self._status_cb, status_qos)
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.goal_approach_pub = self.create_publisher(PoseStamped, "/goal_pose_approach", 10)
        self.goal_direct_pub = self.create_publisher(PoseStamped, "/goal_pose_direct", 10)
        self.goal_carry_direct_pub = self.create_publisher(PoseStamped, "/goal_pose_carry_direct", 10)
        self.goal_carry_clear_direct_pub = self.create_publisher(PoseStamped, "/goal_pose_carry_clear_direct", 10)
        self.goal_carry_planned_pub = self.create_publisher(PoseStamped, "/goal_pose_carry_planned", 10)
        self.goal_correct_heading_pub = self.create_publisher(PoseStamped, "/goal_pose_correct_heading", 10)

    def _status_cb(self, msg):
        self.nav_status = msg.data

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.chassis_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_chassis_yaw = True

    def _write_debug_marker(self, label, x, y):
        # Best-effort only -- a failure here must never abort the actual
        # task, per direct instruction.
        try:
            self._debug_markers[label] = (x, y)
            with open(DEBUG_MARKERS_FILE, "w") as f:
                for lbl, (mx, my) in self._debug_markers.items():
                    f.write(f"{lbl},{mx},{my}\n")
        except Exception as e:
            self.get_logger().warn(f"debug-vis marker write failed ({e}) -- continuing")

    def set_yaw_compensation(self, enabled: bool):
        with open(YAW_COMPENSATION_CONTROL_FILE, "w") as f:
            f.write("on" if enabled else "off")

    def send_correct_heading_and_wait(self, target_yaw, timeout=STATUS_TIMEOUT):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        msg.pose.orientation.w = math.cos(target_yaw / 2.0)
        self.goal_correct_heading_pub.publish(msg)
        if not self.wait_for_status("driving", timeout=5.0):
            self.get_logger().warn("no 'driving' status seen after sending correct-heading goal")
        return self.wait_for_status("reached", timeout=timeout)

    def wait_for_plate_correction(self, timeout=SETTLE_TIMEOUT):
        """Poll the real plate-orientation status (not an assumption that
        the correction goal 'reached' means the plate itself converged --
        'reached' only reflects the chassis's own heading, a separate DOF)
        until it reports ok or this times out."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.plate_orientation_ok():
                return True
            time.sleep(0.1)
        self.get_logger().warn(f"plate did not converge within {timeout}s of the correction goal")
        return False

    def send_goal(self, x, y, mode="normal"):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        pub = {"normal": self.goal_pub, "approach": self.goal_approach_pub, "direct": self.goal_direct_pub,
               "carry": self.goal_carry_direct_pub, "carry_clear": self.goal_carry_clear_direct_pub,
               "carry_planned": self.goal_carry_planned_pub}[mode]
        pub.publish(msg)

    def send_goal_and_wait(self, x, y, mode="normal", timeout=STATUS_TIMEOUT):
        """send_goal + wait_for_status("reached"), but safely -- /nav_status
        is TRANSIENT_LOCAL, so a subscriber connecting to an already-running
        controller (e.g. reused across multiple task runs, not restarted
        each time) immediately receives whatever status is LEFT OVER from
        a previous run. Confirmed live this session: without first waiting
        for a fresh "driving" status (which the controller publishes
        immediately on receiving THIS goal), a leftover "reached" from the
        prior run could be mistaken for this goal already being done,
        skipping the wait -- and the actual drive -- entirely."""
        self.send_goal(x, y, mode)
        if not self.wait_for_status("driving", timeout=5.0):
            self.get_logger().warn("no 'driving' status seen after sending goal -- is rotate_drive_controller.py running?")
        return self.wait_for_status("reached", timeout=timeout)

    def wait_for_status(self, target, timeout=STATUS_TIMEOUT):
        deadline = time.monotonic() + timeout
        warned_stalled = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.nav_status == target:
                return True
            if self.nav_status == "stalled" and target == "reached" and not warned_stalled:
                self.get_logger().warn("nav_status went to 'stalled' while waiting for 'reached' -- still waiting, may recover via replan")
                warned_stalled = True
            elif self.nav_status != "stalled":
                warned_stalled = False
        return False

    def piston_command(self, cmd, wait_status, timeout=60.0):
        try:
            import os
            if os.path.exists(PISTON_STATUS_FILE):
                os.remove(PISTON_STATUS_FILE)
        except OSError:
            pass
        with open(PISTON_CONTROL_FILE, "w") as f:
            f.write(cmd)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                with open(PISTON_STATUS_FILE) as f:
                    if f.read().strip() == wait_status:
                        return True
            except OSError:
                pass
            time.sleep(0.2)
        return False

    def cargo_attached(self):
        """Real check, not an assumption -- reads the bridge's own
        continuous relative-position tracking (see nav2_bridge_robot1.py)
        rather than trusting that a completed lift means the object is
        still there. Per direct user request this session, after a run
        where cargo fell off mid-sequence and the task still reported
        success."""
        try:
            with open(CARGO_STATUS_FILE) as f:
                return f.read().strip().startswith("attached")
        except OSError:
            return False

    def plate_orientation_ok(self):
        """Real check on the plate's actual world orientation, not an
        assumption that apply_yaw_compensation is working -- per direct
        user request this session, after live-catching a bug where the
        counter-rotation was computing correctly but a leftover manual-
        control code path silently overwrote it on the same joint every
        tick, undetected until a carried object visibly swung into
        things."""
        try:
            with open(PLATE_ORIENTATION_STATUS_FILE) as f:
                status = f.read().strip()
                if status.startswith("drifted"):
                    self.get_logger().warn(f"plate orientation check: {status}")
                return status == "ok"
        except OSError:
            return False

    def _read_max_wheel_velocity(self):
        """One round-trip of the bridge's remote diagnostic, parsed for
        the max_abs_wheel_velocity line."""
        import os
        if os.path.exists(DIAG_RESPONSE_FILE):
            os.remove(DIAG_RESPONSE_FILE)
        with open(DIAG_FILE, "w") as f:
            f.write(f"settle-check-{time.monotonic()}")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                with open(DIAG_RESPONSE_FILE) as f:
                    content = f.read()
                for line in content.splitlines():
                    if line.startswith("max_abs_wheel_velocity:"):
                        return float(line.split(":", 1)[1].strip())
            except (OSError, ValueError):
                pass
            time.sleep(0.1)
        return None

    def wait_until_stopped(self, timeout=SETTLE_TIMEOUT):
        """Real check that the chassis has actually stopped moving (real
        wheel velocity, not commanded velocity), not an assumption that
        'reached' means stationary. Added after a live-watched run showed
        the object being lowered while the chassis was still visibly
        coasting under its own momentum -- 'reached' zeroes the commanded
        velocity but doesn't wait out the loaded chassis's real coast."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            max_vel = self._read_max_wheel_velocity()
            if max_vel is not None and max_vel < SETTLE_VELOCITY_TOL:
                return True
            time.sleep(0.1)
        self.get_logger().warn(f"chassis did not settle under {SETTLE_VELOCITY_TOL} rad/s within {timeout}s -- lowering anyway")
        return False

    def _settle_pause(self, seconds=STEP_SETTLE_PAUSE):
        """Fixed pause after a step is confirmed complete, before starting
        the next one -- separate from the real physical-settle checks
        (wait_until_stopped, wait_for_plate_correction), which wait for an
        actual measured condition. This is a flat margin on top of those,
        not a substitute for them."""
        time.sleep(seconds)

    def run(self):
        self.get_logger().info(f"querying {self.object_prim} position...")
        obj_x, obj_y = query_prim_odom_xy(self.object_prim)
        self.get_logger().info(f"{self.object_prim} at odom ({obj_x:.3f}, {obj_y:.3f})")

        self.get_logger().info(f"querying {self.shelf2_prim} real footprint (bbox, not Xform origin)...")
        shelf2_y_min, shelf2_y_max = query_prim_odom_bbox_y(self.shelf2_prim)
        shelf2_y = (shelf2_y_min + shelf2_y_max) / 2.0
        self.get_logger().info(
            f"{self.shelf2_prim} real footprint odom y=[{shelf2_y_min:.3f}, {shelf2_y_max:.3f}] -- targeting center y={shelf2_y:.3f}")

        self.get_logger().info("driving to pickup point (direct, no planning)...")
        # Direct mode: a single turn-to-exact-bearing-then-drive-straight
        # leg (no planner routing, no obstacle check), matching
        # two_robot_pickup_demo.py's original run_goto exactly.
        if not self.send_goal_and_wait(obj_x, obj_y, mode="direct"):
            self.get_logger().error("never reached the pickup point -- aborting")
            return False
        self._settle_pause()

        self.get_logger().info("lifting...")
        if not self.piston_command(f"lift:{self.object_prim}", "lifted"):
            self.get_logger().error("lift never completed -- aborting")
            return False
        if not self.cargo_attached():
            self.get_logger().error(f"{self.object_prim} is not on the plate after lift -- aborting")
            return False
        self._settle_pause()

        # Explicit two-step exit, per direct user correction this session:
        # rotate to face shelf 2 FIRST (a real turn, not folded into a
        # single long drive), then drive only a short clearance distance
        # as its own verifiable step before committing to the full carry
        # -- not continuing along the entry heading (an earlier version's
        # mistake) and not combining the turn with the entire remaining
        # distance in one shot (this version's other earlier mistake).
        # direct mode's turn phase locks onto the bearing toward the
        # clearance point, which is the same bearing as the real shelf-2
        # target (X is identical, so the direction is pure +/-Y) -- so
        # once clear, continuing to the real target needs no further
        # rotation, just more of the same straight line.
        direction = 1.0 if shelf2_y > obj_y else -1.0
        clear_x, clear_y = obj_x, obj_y + direction * EXIT_CLEARANCE_DISTANCE
        self._write_debug_marker("shelf1_exit", clear_x, clear_y)
        # mode="carry_clear", not "direct" -- turns while carrying use
        # CARRY_ANGULAR_SPEED (slower) so the plate's counter-rotation
        # (a real position-target joint, not instantaneous) can actually
        # keep pace with the chassis turn. Confirmed live this session:
        # at the full (empty-handed) turn speed, the carried object
        # visibly swung with the chassis into nearby objects instead of
        # holding its orientation. Obstacle check off for this leg
        # specifically (not "carry", which keeps it on) -- confirmed live
        # this session that the real /scan reading right at the start of
        # this leg (~0.17-0.36m) is shelf 1's own structure, not a real
        # obstacle; the known obstacle (Cube_03) is well past this leg's
        # target and only relevant on the next, longer carry leg.
        self.get_logger().info(f"rotating to face shelf 2 and exiting shelf 1 to ({clear_x:.3f}, {clear_y:.3f})...")
        if not self.send_goal_and_wait(clear_x, clear_y, mode="carry_clear"):
            self.get_logger().error("never cleared shelf 1 -- aborting (still holding object up)")
            return False
        if not self.cargo_attached():
            self.get_logger().error(f"{self.object_prim} came off the plate during the exit turn -- aborting")
            return False
        if not self.plate_orientation_ok():
            self.get_logger().error("plate orientation drifted during the exit turn -- aborting")
            return False
        # Reference for the shelf-2 staging-point correction below: the
        # exact chassis heading leaving shelf 1. Captured here, not
        # assumed -- /odom-fed, live.
        if not self.have_chassis_yaw:
            self.get_logger().error("no /odom received yet -- cannot capture shelf-1-exit heading")
            return False
        shelf1_exit_yaw = self.chassis_yaw
        self._settle_pause()

        # Staging point: ENTRY_CLEARANCE_DISTANCE short of shelf 2's real
        # target, same axis. mode="carry_planned" for THIS leg only -- real
        # planner_server routing + obstacle-triggered replan active, this
        # is the open stretch of corridor where Cube_03 actually sits.
        # Yaw compensation is suspended for this leg specifically -- it's
        # the long, multi-turn detour, and the plate joint isn't torqued
        # to track a rapidly-changing target through several turns in a
        # row (real drift up to ~150deg confirmed live this session). The
        # plate just passively rides along with the chassis until the
        # staging point, where it gets corrected in one unhurried shot
        # instead of continuously fighting to keep up mid-transit.
        self.set_yaw_compensation(False)
        staging_y = shelf2_y - direction * ENTRY_CLEARANCE_DISTANCE
        self._write_debug_marker("shelf2_staging", obj_x, staging_y)
        self.get_logger().info(f"carrying to shelf 2 staging point ({obj_x:.3f}, {staging_y:.3f}) -- collision avoidance active, yaw compensation suspended...")
        reached_staging = self.send_goal_and_wait(obj_x, staging_y, mode="carry_planned")
        if not reached_staging:
            self.set_yaw_compensation(True)
            self.get_logger().error("never reached the shelf 2 staging point -- aborting (still holding object up)")
            return False
        if not self.cargo_attached():
            self.set_yaw_compensation(True)
            self.get_logger().error(f"{self.object_prim} came off the plate during carry -- aborting before lowering")
            return False
        self._settle_pause()

        # Correction: bring the chassis and the plate back to the exact
        # configuration they had leaving shelf 1 -- two independent
        # rotations, done ONE AT A TIME, each fully confirmed complete
        # (plus a settle pause) before the next starts, not run
        # concurrently. Yaw compensation is deliberately still OFF for the
        # chassis rotation below, so the plate isn't fighting to react to
        # the chassis while it's still turning -- it only starts
        # correcting once the chassis has already finished and settled.
        self.get_logger().info(f"correcting chassis heading to shelf-1-exit configuration ({math.degrees(shelf1_exit_yaw):.1f}deg)...")
        if not self.send_correct_heading_and_wait(shelf1_exit_yaw):
            self.set_yaw_compensation(True)
            self.get_logger().error("chassis heading correction never completed -- aborting")
            return False
        self._settle_pause()

        self.get_logger().info("correcting plate orientation to shelf-1-exit configuration...")
        self.set_yaw_compensation(True)
        if not self.wait_for_plate_correction():
            self.get_logger().error("plate orientation correction never converged -- aborting before lowering")
            return False
        self._settle_pause()
        if not self.cargo_attached():
            self.get_logger().error(f"{self.object_prim} came off the plate during the heading correction -- aborting")
            return False

        # Final entry into shelf 2: obstacle check off again, same reasoning
        # as the exit-clearance leg -- the destination is right next to
        # shelf 2's own structure by design.
        self._write_debug_marker("shelf2_entry", obj_x, shelf2_y)
        self.get_logger().info(f"entering shelf 2, dropping off at ({obj_x:.3f}, {shelf2_y:.3f})...")
        if not self.send_goal_and_wait(obj_x, shelf2_y, mode="carry_clear"):
            self.get_logger().error("never reached shelf 2 -- aborting (still holding object up)")
            return False
        if not self.cargo_attached():
            self.get_logger().error(f"{self.object_prim} came off the plate during carry -- aborting before lowering")
            return False
        if not self.plate_orientation_ok():
            self.get_logger().error("plate orientation drifted during carry -- aborting before lowering")
            return False
        self._settle_pause()

        self.wait_until_stopped()
        self._settle_pause()
        self.get_logger().info("lowering...")
        if not self.piston_command("lower", "lowered"):
            self.get_logger().error("lower never completed")
            return False
        self._settle_pause()

        self.get_logger().info("transfer complete")
        return True


def main():
    object_prim = sys.argv[1] if len(sys.argv) > 1 else "/Cube"
    shelf2_prim = sys.argv[2] if len(sys.argv) > 2 else "/World/Shelf2"
    rclpy.init()
    node = ShelfTransferTask(object_prim, shelf2_prim)
    # Let ROS2 discovery actually settle before sending anything -- the
    # goal-topic publishers created in __init__ aren't necessarily
    # matched to rotate_drive_controller.py's subscribers yet, and that
    # matching only gets processed by spinning, not by plain time.sleep()
    # (which the position-query wait loops use). Confirmed live this
    # session: without this, the very first send_goal() was silently
    # dropped -- published before the publisher knew about the
    # subscriber -- and the controller never saw it (stayed "idle").
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        ok = node.run()
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
