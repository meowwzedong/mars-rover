import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'prototype_1'

    # 1. Read the URDF file
    xacro_file = os.path.join(
        get_package_share_directory(package_name), 'urdf', 'prototype_1.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # 2. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 3. Include Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. Spawn the Robot Entity in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'prototype_1'],
        output='screen'
    )

    # 5. Spawner for Joint State Broadcaster (Crucial for Arm Position tracking)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # 6. Spawner for Arm Controller (This activates the PID muscles!)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        joint_state_broadcaster_spawner,  # Turns on tracking
        arm_controller_spawner            # Turns on arm motors
    ])