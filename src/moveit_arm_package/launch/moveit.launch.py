"""MoveIt move_group + RViz, wired to the Gazebo-spawned controllers.

Run AFTER gazebo.launch.py. This does NOT start a ros2_control_node or any
mock hardware — execution goes through moveit_simple_controller_manager
(moveit_controllers.yaml -> arm_controller/follow_joint_trajectory), which is
the JointTrajectoryController that gazebo_ros2_control spawned in the sim.

use_sim_time=True so move_group runs on Gazebo's /clock and trajectory timing
matches the simulated controllers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm_robot", package_name="moveit_config"
    ).to_moveit_configs()

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz_config = os.path.join(
        get_package_share_directory("moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
