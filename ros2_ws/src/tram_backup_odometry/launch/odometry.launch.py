from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = Path(get_package_share_directory('tram_backup_odometry')) / 'config' / 'params.yaml'
    return LaunchDescription([
        Node(package='tram_backup_odometry', executable='odometry', name='tram_backup_odometry',
             parameters=[str(params)], output='screen'),
    ])
