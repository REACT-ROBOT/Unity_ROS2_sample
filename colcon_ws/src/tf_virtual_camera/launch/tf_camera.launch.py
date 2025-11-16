#!/usr/bin/env python3
"""
Launch file for TF Virtual Camera Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    link_csv_file_arg = DeclareLaunchArgument(
        'link_csv_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tf_virtual_camera'),
            'config',
            'khr3_links.csv'
        ]),
        description='CSV file containing link names for both robots'
    )

    robot1_name_arg = DeclareLaunchArgument(
        'robot1_name',
        default_value='KHR3_1',
        description='Name of the first robot'
    )

    robot2_name_arg = DeclareLaunchArgument(
        'robot2_name',
        default_value='KHR3_2',
        description='Name of the second robot'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='Reference frame for TF lookups'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Update rate in Hz'
    )

    enable_virtual_cam_arg = DeclareLaunchArgument(
        'enable_virtual_cam',
        default_value='false',
        description='Enable virtual camera output (Linux only)'
    )

    enable_network_stream_arg = DeclareLaunchArgument(
        'enable_network_stream',
        default_value='true',
        description='Enable network stream output (for WSL2/remote)'
    )

    stream_host_arg = DeclareLaunchArgument(
        'stream_host',
        default_value='auto',
        description='Network stream destination host (auto=detect Windows host in WSL2)'
    )

    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='Network stream destination port'
    )

    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='false',
        description='Show OpenCV preview window (requires DISPLAY)'
    )

    # Node
    tf_camera_node = Node(
        package='tf_virtual_camera',
        executable='tf_camera_node',
        name='tf_virtual_camera',
        output='screen',
        parameters=[{
            'link_csv_file': LaunchConfiguration('link_csv_file'),
            'robot1_name': LaunchConfiguration('robot1_name'),
            'robot2_name': LaunchConfiguration('robot2_name'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'update_rate': LaunchConfiguration('update_rate'),
            'grid_width': 80,
            'grid_height': 45,
            'enable_virtual_cam': LaunchConfiguration('enable_virtual_cam'),
            'enable_network_stream': LaunchConfiguration('enable_network_stream'),
            'stream_host': LaunchConfiguration('stream_host'),
            'stream_port': LaunchConfiguration('stream_port'),
            'show_preview': LaunchConfiguration('show_preview'),
        }]
    )

    return LaunchDescription([
        link_csv_file_arg,
        robot1_name_arg,
        robot2_name_arg,
        reference_frame_arg,
        update_rate_arg,
        enable_virtual_cam_arg,
        enable_network_stream_arg,
        stream_host_arg,
        stream_port_arg,
        show_preview_arg,
        tf_camera_node,
    ])
