"""Run the custom arm_controller (MoveGroupInterface) node with MoveIt params.

`ros2 run moveit_arm_package arm_controller` fails because the node can't find
robot_description / robot_description_semantic. This launch injects them as node
parameters. Run it AFTER gazebo.launch.py and moveit.launch.py are up.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm_robot", package_name="moveit_config"
    ).to_moveit_configs()

    arm_controller_node = Node(
        package="moveit_arm_package",
        executable="arm_controller",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([arm_controller_node])
