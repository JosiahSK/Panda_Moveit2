import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from panda_moveit2_tutorials.moveit_config_helper import get_moveit_config_dict


def generate_launch_description():
    """Launch a selected MoveIt 2 Python tutorial node with full configuration parameters."""

    tutorial_arg = DeclareLaunchArgument(
        'tutorial_node',
        default_value='basic_motion_planner',
        description='Name of the tutorial node executable to run (e.g. basic_motion_planner, pose_goal_planner, cartesian_path_planner, collision_avoidance_demo, constrained_motion_demo, pick_and_place, planner_benchmark)'
    )

    moveit_config_dict = get_moveit_config_dict()

    tutorial_node = Node(
        package='panda_moveit2_tutorials',
        executable=LaunchConfiguration('tutorial_node'),
        name=LaunchConfiguration('tutorial_node'),
        output='screen',
        parameters=[moveit_config_dict],
    )

    return LaunchDescription([
        tutorial_arg,
        tutorial_node,
    ])
