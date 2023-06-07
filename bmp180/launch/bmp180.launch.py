from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os
from ament_index_python import get_package_share_directory
import time


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bmp180'),
        'config',
        'bmp180.yaml'
        )
    return LaunchDescription([
        Node(
            package = 'bmp180',
            executable = 'bmp180',
            parameters = [config]
        ),
    ])
