#!/usr/bin/env python3
"""
four_wheeled_robot_keyboard_controller.py

Drive the robot and its arm live with the keyboard using mode switching.
Publishes geometry_msgs/Twist to /cmd_vel and trajectory_msgs/JointTrajectory
to /arm_controller/joint_trajectory.

Controls:
    m : Toggle between BASE and ARM control modes

  [BASE Mode Controls]
    Arrow Up/Down    : Forward / Backward
    Arrow Left/Right : Turn Left / Turn Right
    space / x        : Stop base movement

  [ARM Mode Controls]
    Arrow Up/Down    : Shoulder Pitch (Up / Down)
    Arrow Left/Right : Shoulder Yaw (Left / Right)
    w / s            : Elbow Pitch (Forward / Backward)
    a / d            : Wrist Pitch (Up / Down)
    space / x        : Reset arm joints to 0.0

  [Common]
    q / Ctrl-C       : Quit
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Define key codes for arrow keys
ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'

HELP = """
---------------------------------------------------------
Control Modes: Press 'm' to switch between BASE and ARM.
Current Mode: BASE
---------------------------------------------------------
[BASE MODE]
  Arrow Keys: Move & Turn
  Spacebar / 'x': Stop Base

[ARM MODE]
  Arrow Up/Down   : Shoulder Pitch
  Arrow Left/Right: Shoulder Yaw
  W / S           : Elbow Pitch
  A / D           : Wrist Pitch
  Spacebar / 'x'  : Reset Arm Targets to 0

Common:
  'q' or Ctrl-C   : Quit
---------------------------------------------------------
"""


class KeyboardController(Node):
    def __init__(self):
        super().__init__('prototype_controller')
        
        # Publishers
        self.base_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Control Mode state ('BASE' or 'ARM')
        self.mode = 'BASE'

        # Current commanded base velocities
        self.linear = 0.0
        self.angular = 0.0

        # Step sizes for base
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 1.0   # rad/s

        # Current target angles for arm joints (in radians)
        self.shoulder_yaw = 0.0
        self.shoulder_pitch = 0.0
        self.elbow = 0.0
        self.wrist = 0.0

        # Step size for arm joints (~5 degrees per key press)
        self.joint_step = 0.087

    def update_from_key(self, key):
        """Return False if the user asked to quit, True otherwise."""
        
        # Toggle control mode
        if key.lower() == 'm':
            self.stop_robot() # Safely zero out velocities before switching
            self.mode = 'ARM' if self.mode == 'BASE' else 'BASE'
            self.get_logger().info(f"Switched control mode to: {self.mode}")
            return True

        # Global Quit commands
        if key in ('q', '\x03'):   # q or Ctrl-C
            return False

        # --- BASE MODE LOGIC ---
        if self.mode == 'BASE':
            if key == ARROW_UP:
                self.linear = self.linear_speed
            elif key == ARROW_DOWN:
                self.linear = -self.linear_speed
            elif key == ARROW_LEFT:
                self.angular = self.angular_speed
            elif key == ARROW_RIGHT:
                self.angular = -self.angular_speed
            elif key in (' ', 'x'):
                self.linear = 0.0
                self.angular = 0.0

        # --- ARM MODE LOGIC ---
        elif self.mode == 'ARM':
            # Shoulder Controls (Arrows)
            if key == ARROW_LEFT:
                self.shoulder_yaw += self.joint_step
            elif key == ARROW_RIGHT:
                self.shoulder_yaw -= self.joint_step
            elif key == ARROW_UP:
                self.shoulder_pitch += self.joint_step
            elif key == ARROW_DOWN:
                self.shoulder_pitch -= self.joint_step
            
            # Elbow Controls (W / S)
            elif key.lower() == 'w':
                self.elbow += self.joint_step
            elif key.lower() == 's':
                self.elbow -= self.joint_step
            
            # Wrist Controls (A / D)
            elif key.lower() == 'a':
                self.wrist += self.joint_step
            elif key.lower() == 'd':
                self.wrist -= self.joint_step
            
            # Reset Arm Position
            elif key in (' ', 'x'):
                self.shoulder_yaw = 0.0
                self.shoulder_pitch = 0.0
                self.elbow = 0.0
                self.wrist = 0.0

        return True

    def publish(self):
        # 1. Always publish base commands to keep diff_drive active or stopped
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear)
        twist_msg.angular.z = float(self.angular)
        self.base_publisher.publish(twist_msg)

        # 2. Always publish arm target configurations to the trajectory controller
        arm_msg = JointTrajectory()
        arm_msg.joint_names = [
            'shoulder_yaw_joint', 
            'shoulder_pitch_joint', 
            'elbow_joint', 
            'wrist_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [
            float(self.shoulder_yaw), 
            float(self.shoulder_pitch), 
            float(self.elbow), 
            float(self.wrist)
        ]
        # Direct the controller to reach this position smoothly within 100ms
        point.time_from_start = Duration(sec=0, nanosec=10000000) 
        
        arm_msg.points.append(point)
        self.arm_publisher.publish(arm_msg)

    def stop_robot(self):
        self.linear = 0.0
        self.angular = 0.0
        # Create an immediate flat message to stop mobile base base
        twist_msg = Twist()
        self.base_publisher.publish(twist_msg)


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
            # Continuously stream states to Gazebo 
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