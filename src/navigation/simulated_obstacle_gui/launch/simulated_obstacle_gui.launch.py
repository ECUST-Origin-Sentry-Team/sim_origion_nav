"""Launch the simulated obstacle GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the GUI node with overridable topic and frame parameters."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument(
            'center_topic', default_value='/simulated_obstacle/center'),
        DeclareLaunchArgument(
            'enabled_topic', default_value='/simulated_obstacle/enabled'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('clear_on_exit', default_value='true'),
        Node(
            package='simulated_obstacle_gui',
            executable='simulated_obstacle_gui',
            name='simulated_obstacle_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('frame_id'),
                'center_topic': LaunchConfiguration('center_topic'),
                'enabled_topic': LaunchConfiguration('enabled_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'clear_on_exit': LaunchConfiguration('clear_on_exit'),
            }],
        ),
    ])
