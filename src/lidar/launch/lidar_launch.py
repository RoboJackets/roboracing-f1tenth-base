import os
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    lidar_launch_file = os.path.join(
        get_package_share_directory("urg_node2"), "launch", "urg_node2.launch.py"
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file)
    )

    odom_dummy_node = Node(
        package='lidar',
        namespace='lidar',
        executable='lidarnode',
        name='lidarnode'
    )

    slam_toolbox_file = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_sync_launch.py"
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_file)
    )

    return LaunchDescription([
        lidar,
        odom_dummy_node,
        slam_toolbox
    ])