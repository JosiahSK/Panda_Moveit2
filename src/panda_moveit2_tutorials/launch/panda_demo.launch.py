from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch MoveIt 2 demo with the Panda robot in RViz."""

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # NOTE: The main MoveIt demo is launched via:
        # ros2 launch moveit_resources_panda_moveit_config demo.launch.py
        # This launch file is for running tutorial nodes alongside it.
        
        Node(
            package='panda_moveit2_tutorials',
            executable='basic_motion_planner',
            name='basic_motion_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
