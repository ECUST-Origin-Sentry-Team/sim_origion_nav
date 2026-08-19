from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pcd_file = LaunchConfiguration('pcd_file')
    cloud_registered_topic = LaunchConfiguration('cloud_registered_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_localization = Node(
        package='small_gicp_localization',
        executable='small_gicp_localization_node',
        name='small_gicp_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_threads': 4,
            'num_neighbors': 10,
            'global_leaf_size': 0.25,
            'registered_leaf_size': 0.25,
            'max_dist_sq': 1.0,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'aft_mapped',
            'pcd_file': pcd_file,
            'cloud_registered_topic': cloud_registered_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_file',
            default_value='',
            description='Absolute path to the PCD map used by GICP',
        ),
        DeclareLaunchArgument(
            'cloud_registered_topic',
            default_value='/cloud_registered',
            description='Point cloud topic; Point-LIO publishes it in the odom frame',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        start_localization,
    ])
