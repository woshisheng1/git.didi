#!/usr/bin/python3
import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    
    dipan_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('origincar_base'),'/launch/','origincar_bringup.launch.py']
        )
    )
    leida_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('lslidar_driver'),'/launch/','lsn10p_launch.py']
        )
    )
    rviz_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('lslidar_driver'),'/launch/','viewer_scan_launch.py']
        )
    )
    TF_node = launch.actions.ExecuteProcess(
        cmd=['ros2','run','tf2_ros','static_transform_publisher','0.08','0.0','0.15','0.0','0.0','0.0','base_footprint','laser']
    )


    return launch.LaunchDescription([dipan_launch,leida_launch,rviz_launch,TF_node])
