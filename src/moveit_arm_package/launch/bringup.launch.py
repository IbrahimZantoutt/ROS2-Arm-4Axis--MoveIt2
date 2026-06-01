"""Full bring-up: Gazebo simulation + MoveIt move_group + RViz in one shot.

Includes the existing gazebo.launch.py and moveit.launch.py. move_group is
delayed a few seconds so Gazebo and its controller_manager are up first
(move_group connects to the spawned controllers on start).

After this is running, command the arm with:
    ros2 launch moveit_arm_package run_controller.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_arm = get_package_share_directory("moveit_arm_package")
    launch_dir = os.path.join(pkg_arm, "launch")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "gazebo.launch.py"))
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit.launch.py"))
    )

    # Give Gazebo + controller_manager a head start before move_group/RViz.
    delayed_moveit = TimerAction(period=5.0, actions=[moveit])

    return LaunchDescription([gazebo, delayed_moveit])
