#!/usr/bin/env python3
"""
four_wheeled_robot_controller.py

Drive the robot's BASE with the keyboard, and drive the ARM in CARTESIAN space
using analytical inverse kinematics (IK). Instead of nudging individual joints,
you jog a target point (x, y, z) for the gripper tip; the node solves the joint
angles that put the tip there and streams them to the trajectory controller.

Publishes:
    geometry_msgs/Twist          -> /cmd_vel
    trajectory_msgs/JointTrajectory -> /arm_controller/joint_trajectory

------------------------------------------------------------------
HOW THE IK WORKS (matches this robot's URDF):

  Shoulder-pitch axis point S = (0.15, 0, 0.12) in base_link.
  Link lengths (along the arm):
      L1 = 0.20  shoulder_pitch -> elbow
      L2 = 0.16  elbow          -> wrist
      L3 = 0.08  wrist          -> gripper tip

  shoulder_yaw   (Z axis) -> picks the vertical plane:  yaw = atan2(dy, dx)
  shoulder_pitch (Y axis) }
  elbow          (Y axis) } -> a planar arm inside that plane
  wrist          (Y axis) }

  We hold the gripper at a fixed approach angle 'phi' (angle of the last link
  from vertical). Subtracting that fixed last link from the target gives the
  WRIST point, which we solve with classic law-of-cosines 2-link IK.
------------------------------------------------------------------

Controls:
    m : Toggle between BASE and ARM control modes
    q / Ctrl-C : Quit

  [BASE Mode]
    Arrow Up/Down    : Forward / Backward
    Arrow Left/Right : Turn Left / Turn Right
    space / x        : Stop base

  [ARM Mode]  (Cartesian jogging of the gripper-tip target)
    Arrow Up/Down    : +x / -x  (reach forward / back)
    Arrow Left/Right : +y / -y  (move target left / right)
    w / s            : +z / -z  (raise / lower target)
    a / d            : tilt gripper approach angle up / down
    e                : toggle elbow-up / elbow-down solution
    space / x        : reset target to home pose
"""

import sys
import math
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Arrow key escape codes
ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'

HELP = """
---------------------------------------------------------
Press 'm' to switch between BASE and ARM. Start mode: BASE
---------------------------------------------------------
[BASE MODE]
  Arrow Keys     : Move & Turn
  space / 'x'    : Stop base

[ARM MODE]  (jog the gripper-tip target in meters)
  Arrow Up/Down  : +x / -x   (reach forward / back)
  Arrow L/R      : +y / -y   (target left / right)
  W / S          : +z / -z   (raise / lower)
  A / D          : tilt gripper up / down (approach angle)
  E              : toggle elbow up / down
  space / 'x'    : reset target to home

Common:
  'q' or Ctrl-C  : Quit
---------------------------------------------------------
"""


class KeyboardController(Node):
    def __init__(self):
        super().__init__('four_wheeled_robot_controller')

        # ---- Publishers ----
        self.base_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.mode = 'BASE'

        # ---- Base velocity state ----
        self.linear = 0.0
        self.angular = 0.0
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 1.0   # rad/s
        self.gripper = 0.0        # 0 = open, positive = closing
        self.gripper_step = 0.002

        self.last_base_key_time = 0.0
        self.base_timeout = 0.6   # seconds without a key -> stop

        # ======================================================
        #  ARM IK GEOMETRY  (read straight from the URDF)
        # ======================================================
        # Shoulder-pitch axis point in base_link frame:
        #   arm_base_joint (0.15,0,0.05) + shoulder_yaw (0,0,0.05)
        #   + shoulder_pitch offset (0,0,0.02) = (0.15, 0, 0.12)
        self.Sx, self.Sy, self.Sz = 0.15, 0.0, 0.12
        self.L1 = 0.20   # shoulder_pitch -> elbow
        self.L2 = 0.16   # elbow          -> wrist
        self.L3 = 0.08   # wrist          -> gripper tip

        # Joint limits (from URDF). yaw is continuous; clamp to +/-pi.
        self.lim_yaw   = (-math.pi, math.pi)
        self.lim_pitch = (-1.5708, 1.5708)
        self.lim_elbow = (-2.0, 2.0)
        self.lim_wrist = (-2.0, 2.0)

        # ---- Cartesian target (gripper tip) in base_link frame ----
        self.home_target = (0.43, 0.0, 0.32)   # a comfortable, reachable pose
        self.home_phi = math.pi / 2.0          # gripper pointing horizontally out
        self.tx, self.ty, self.tz = self.home_target
        self.phi = self.home_phi               # approach angle from vertical
        self.elbow_up = True                   # which IK branch to use

        # Jog step sizes
        self.pos_step = 0.01    # 1 cm per press
        self.phi_step = 0.087   # ~5 deg per press

        # Last good joint solution (so we always publish something valid)
        self.joints = self.solve_ik(self.tx, self.ty, self.tz,
                                    self.phi, self.elbow_up)
        if self.joints is None:
            # Should not happen for the home pose, but be safe.
            self.joints = (0.0, 0.0, 0.0, 0.0)

    # ----------------------------------------------------------
    #  ANALYTICAL INVERSE KINEMATICS
    #  Returns (yaw, shoulder_pitch, elbow, wrist) or None if the
    #  target is unreachable / would violate a joint limit.
    # ----------------------------------------------------------
    def solve_ik(self, x, y, z, phi, elbow_up):
        # 1) Position of target relative to shoulder point S
        dx = x - self.Sx
        dy = y - self.Sy
        dz = z - self.Sz

        # 2) Yaw picks the vertical working plane
        yaw = math.atan2(dy, dx)
        r = math.hypot(dx, dy)          # horizontal reach (>= 0)

        # 3) Back off the fixed last link to get the WRIST point,
        #    expressed in the planar (radial r, vertical z-from-S) frame.
        #    Angles here are measured FROM VERTICAL, positive tilting outward.
        wr = r - self.L3 * math.sin(phi)
        wz = dz - self.L3 * math.cos(phi)

        dist2 = wr * wr + wz * wz
        dist = math.sqrt(dist2)

        # 4) Reachability of the 2-link (L1, L2) sub-arm
        if dist > (self.L1 + self.L2) or dist < abs(self.L1 - self.L2):
            return None

        # 5) Elbow angle via law of cosines
        cos_t2 = (dist2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))   # guard tiny float overshoot
        t2 = math.acos(cos_t2)                  # magnitude in [0, pi]
        if not elbow_up:
            t2 = -t2

        # 6) Shoulder pitch (absolute angle from vertical = beta1)
        k1 = self.L1 + self.L2 * math.cos(t2)
        k2 = self.L2 * math.sin(t2)
        beta1 = math.atan2(wr, wz) - math.atan2(k2, k1)

        sp = beta1                       # shoulder_pitch
        el = t2                          # elbow (relative)
        wri = phi - (beta1 + t2)         # wrist closes the chain to reach phi

        # 7) Reject if any joint is outside its limit
        joints = (yaw, sp, el, wri)
        limits = (self.lim_yaw, self.lim_pitch, self.lim_elbow, self.lim_wrist)
        for val, (lo, hi) in zip(joints, limits):
            if val < lo or val > hi:
                return None

        return joints

    def try_set_target(self, tx, ty, tz, phi, elbow_up):
        """Solve IK for a candidate target; commit it only if it succeeds."""
        sol = self.solve_ik(tx, ty, tz, phi, elbow_up)
        if sol is None:
            self.get_logger().warn("Target unreachable / joint limit - ignored.")
            return
        self.tx, self.ty, self.tz = tx, ty, tz
        self.phi = phi
        self.elbow_up = elbow_up
        self.joints = sol
        yaw, sp, el, wri = sol
        self.get_logger().info(
            f"tip=({tx:.3f},{ty:.3f},{tz:.3f}) phi={math.degrees(phi):.0f} | "
            f"yaw={math.degrees(yaw):.0f} sp={math.degrees(sp):.0f} "
            f"el={math.degrees(el):.0f} wr={math.degrees(wri):.0f}"
        )

    def update_from_key(self, key):
        """Return False if the user asked to quit, True otherwise."""

        if key.lower() == 'm':
            self.stop_robot()
            self.mode = 'ARM' if self.mode == 'BASE' else 'BASE'
            self.get_logger().info(f"Switched control mode to: {self.mode}")
            return True

        if key in ('q', '\x03'):   # q or Ctrl-C
            return False

        # ---------------- BASE MODE ----------------
        if self.mode == 'BASE':
            if key == ARROW_UP:
                self.linear = self.linear_speed
                self.last_base_key_time = time.time()
            elif key == ARROW_DOWN:
                self.linear = -self.linear_speed
                self.last_base_key_time = time.time()
            elif key == ARROW_LEFT:
                self.angular = self.angular_speed
                self.last_base_key_time = time.time()
            elif key == ARROW_RIGHT:
                self.angular = -self.angular_speed
                self.last_base_key_time = time.time()
            elif key in (' ', 'x'):
                self.linear = 0.0
                self.angular = 0.0
            return True
        # ---------------- ARM MODE (Cartesian IK) ----------------
        tx, ty, tz, phi, eu = self.tx, self.ty, self.tz, self.phi, self.elbow_up

        if key == ARROW_UP:
            tx += self.pos_step
        elif key == ARROW_DOWN:
            tx -= self.pos_step
        elif key == ARROW_LEFT:
            ty += self.pos_step
        elif key == ARROW_RIGHT:
            ty -= self.pos_step
        elif key.lower() == 'w':
            tz += self.pos_step
        elif key.lower() == 's':
            tz -= self.pos_step
        elif key.lower() == 'a':
            phi += self.phi_step
        elif key.lower() == 'd':
            phi -= self.phi_step
        elif key.lower() == 'e':
            eu = not eu
        elif key.lower() == 'c':   # close
            self.gripper = min(self.gripper - self.gripper_step, 0.02)
        elif key.lower() == 'o':   # open
            self.gripper = max(self.gripper + self.gripper_step, 0.0)
        # Reset Arm Position
        elif key in (' ', 'x'):
            tx, ty, tz = self.home_target
            phi = self.home_phi
            eu = True
            self.gripper = 0.0
        else:
            return True

        self.try_set_target(tx, ty, tz, phi, eu)
        return True

    def publish(self):
        # Base
        if self.mode == 'BASE' and (self.linear or self.angular):
            if time.time() - self.last_base_key_time > self.base_timeout:
                self.linear = 0.0
                self.angular = 0.0
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear)
        twist_msg.angular.z = float(self.angular)
        self.base_publisher.publish(twist_msg)

        # Arm: stream the IK joint solution
        arm_msg = JointTrajectory()
        arm_msg.joint_names = [
            'shoulder_yaw_joint',
            'shoulder_pitch_joint',
            'elbow_joint',
            'wrist_joint',
            'gripper_left_joint',
            'gripper_right_joint',
        ]
        point = JointTrajectoryPoint()
        yaw, sp, el, wri = self.joints
        point.positions = [
            float(yaw), float(sp), float(el), float(wri),
            float(self.gripper), float(self.gripper)
        ]
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100 ms
        arm_msg.points.append(point)
        self.arm_publisher.publish(arm_msg)

    def stop_robot(self):
        self.linear = 0.0
        self.angular = 0.0
        self.base_publisher.publish(Twist())


def get_key(settings, timeout=0.1):
    """Read a single keypress without waiting for Enter (non-blocking)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key:
                if not node.update_from_key(key):
                    break
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()