from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the official Panda MoveIt 2 demo (move_group, RViz, robot_state_publisher, controllers)."""

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='moveit.rviz',
        description='RViz configuration file name in moveit_resources_panda_moveit_config/launch'
    )

    panda_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('moveit_resources_panda_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ),
        launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items()
    )

    return LaunchDescription([
        rviz_config_arg,
        panda_demo_launch,
    ])
