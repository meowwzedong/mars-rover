mars-rover prototype 1



This is a four wheeled mobile robot with a 4 DOF robotic arm. It is simulated in Gazebo and controlled by the keyboard through
ros2 humble. The mobile base uses a differential-drive (skid-steer) plugin; the arm is driven by ros2_control through a 
JointTrajectoryController. A single keyboard node teleoperates both, switching between BASE and ARM control modes.


Features

Differential-drive (4-wheel skid-steer) mobile base
4-DOF arm: shoulder yaw, shoulder pitch, elbow, wrist
Keyboard teleop with BASE / ARM mode switching
Position-controlled arm via JointTrajectoryController
Joint state feedback and TF publishing for visualization


Tested Environment

It has only been tested in Ubuntu-22.04 using ROS2 Humble and the simulator used was Gazebo classic 11.


Dependencies 

ROS2 control and gazebo packages that needs to be installed:
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-xacro


Clone and Build

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/meowwzedong/mars-rover.git .
cd ~/ros2_ws
colcon build
source install/setup.bash


Launch 

ros2 launch prototype_1 launch_robot.py


Drive

ros2 run prototype_1 four_wheeled_robot_controller
