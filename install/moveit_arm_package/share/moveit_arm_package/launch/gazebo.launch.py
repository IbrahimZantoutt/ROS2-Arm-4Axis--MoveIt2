"""Bring up the arm in Gazebo Classic with ros2_control.

Sequence:
  1. Start Gazebo (gzserver + gzclient).
  2. Publish robot_description (from arm.urdf.xacro) via robot_state_publisher.
  3. Spawn the robot into Gazebo -> this starts the gazebo_ros2_control
     controller_manager (which reads config/controllers.yaml).
  4. Spawn controllers in order: joint_state_broadcaster, then the
     arm/gripper trajectory controllers.

NOTE: this loads ONLY arm.urdf.xacro (the GazeboSystem description). Do not
also load the moveit_config description in the same session or you get two
ros2_control blocks claiming the same joints.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_arm = get_package_share_directory("moveit_arm_package")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(pkg_arm, "urdf", "arm.urdf.xacro")
    robot_description = {"robot_description": xacro.process_file(xacro_file).toxml()}

    world_file = os.path.join(pkg_arm, "worlds", "blocks.world")

    # Make model://aruco_marker (the marker's material script + texture)
    # resolvable by Gazebo. Append to any existing GAZEBO_MODEL_PATH.
    models_dir = os.path.join(pkg_arm, "models")
    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        f"{models_dir}:{gazebo_model_path}" if gazebo_model_path else models_dir,
    )

    # 1. Gazebo (load our world with the blocks instead of the empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"verbose": "false", "world": world_file}.items(),
    )

    # 2. robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 3. Spawn the robot from the /robot_description topic
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "arm_robot"],
        output="screen",
    )

    # 4. Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            set_model_path,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            # Spawn JSB only after the robot (and its controller_manager) exists.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            # Then the trajectory controllers.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner, gripper_controller_spawner],
                )
            ),
        ]
    )
