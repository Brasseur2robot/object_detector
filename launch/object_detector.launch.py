from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('object_detector'),
        'config',
        'config.yaml'
    )

    launch_file_arg = DeclareLaunchArgument(
        'ldlidar_stl_ros2_launch_file',
        default_value='stl27l.launch.py'
    )

    ldlidar_stl_ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ldlidar_stl_ros2'),
                'launch',
                LaunchConfiguration('ldlidar_stl_ros2_launch_file')
            ])
        )
    )

    object_detector_node = Node(
        package='object_detector',
        executable='object_detector',
        name='object_detector',
        parameters=[config_file]
    )

    uart_sender_node = Node(
        package='object_detector',
        executable='uart_sender',
        name='uart_sender',
        parameters=[config_file]
    )

    return LaunchDescription([
        launch_file_arg,
        ldlidar_stl_ros2_launch,
        object_detector_node,
        uart_sender_node
    ])
