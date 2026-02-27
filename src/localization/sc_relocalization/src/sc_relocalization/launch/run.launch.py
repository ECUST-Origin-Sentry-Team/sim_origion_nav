import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    sc_pkg_share = get_package_share_directory('sc_relocalization')

    default_map_path = "/home/tony/ros/origin_nav2025_developing/map_scantext.pcd_db"

    # 默认 config 路径
    default_config_path = os.path.join(
        sc_pkg_share, 'config', 'relocalization.yaml'
    )

    # -------- Launch Arguments --------
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Absolute path to the map database directory'
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to relocalization config file'
    )

    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.0')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.0')
    initial_z_arg = DeclareLaunchArgument('initial_z', default_value='0.0')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    use_init_arg = DeclareLaunchArgument(
        'use_initial_pose_from_param',
        default_value='true'
    )

    # -------- Node --------
    sc_node = Node(
        package='sc_relocalization',
        executable='sc_relocalization_node',
        name='sc_relocalization_node',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'config_path': LaunchConfiguration('config_path'),
            'use_initial_pose_from_param': LaunchConfiguration('use_initial_pose_from_param'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_z': LaunchConfiguration('initial_z'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }]
    )

    return LaunchDescription([
        map_path_arg,
        config_path_arg,
        initial_x_arg,
        initial_y_arg,
        initial_z_arg,
        initial_yaw_arg,
        use_init_arg,
        sc_node
    ])
