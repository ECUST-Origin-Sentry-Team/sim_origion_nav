from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 定义参数
    project = "adaptive_lio"
    rviz = LaunchConfiguration('rviz', default='true')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Set "true" to launch rviz')

    # Adaptive LIO 节点
    adaptive_lio_node = Node(
        package=project,
        executable=project,
        name=project,
        output='screen',
        emulate_tty=True,
        parameters=[],
        arguments=[]
    )

    # RVIZ 可视化节点
    rviz_group = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
           # arguments=['-d', '/home/lin/adaptive-lio/src/Adaptive-LIO/launch/rviz.rviz'],
            condition=IfCondition(rviz)
        )
    ])

    # 创建launch描述
    ld = LaunchDescription()

    # 添加声明
    ld.add_action(declare_rviz_cmd)

    # 添加节点
    ld.add_action(adaptive_lio_node)
   # ld.add_action(rviz_group)

    return ld