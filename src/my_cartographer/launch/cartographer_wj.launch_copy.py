# cartographer_pure_laser.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('my_cartographer')

    # 定义启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid map'
    )
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Occupancy grid publishing period in seconds'
    )

    # 节点1: Cartographer SLAM 节点
    cartographer_node = Node(
        package='my_cartographer',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', 'Cartographer_2d_copy.lua'
        ],
        remappings=[
            # 假设Fishbot的激光雷达话题是 /scan，确保这里正确映射
            ('scan', '/scan'),
            # 如果你有多个雷达或话题名不同，在此处重映射
        ]
    )

    # 节点2: 占据栅格地图生成节点
    occupancy_grid_node = Node(
        package='my_cartographer',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': LaunchConfiguration('resolution')},
            {'publish_period_sec': LaunchConfiguration('publish_period_sec')}
        ],
        arguments=['-resolution', LaunchConfiguration('resolution'),
                   '-publish_period_sec', LaunchConfiguration('publish_period_sec')]
    )

    # 节点3: RViz 可视化节点 (可选，但强烈推荐)
    # 首先，定位RViz配置文件。我们可以内嵌配置，或指向一个文件。
    # 这里我们启动一个空的RViz，手动加载配置。
    rviz_config_path = os.path.join(pkg_share, 'config', 'robot_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 组装启动描述
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(resolution_arg)
    ld.add_action(publish_period_sec_arg)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld