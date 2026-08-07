"""ROS-TCP-Endpoint と適合性テストをまとめて起動する launch。

シミュレータ本体 (Unity アプリ) の起動は含まない。エンドポイント -> シミュレータ
-> テストの順に立ち上げる必要があるため、全部込みで回したいときは
``scripts/service_conformance_test.sh`` を使うこと。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    junit = LaunchConfiguration('junit')
    start_endpoint = LaunchConfiguration('start_endpoint')

    endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{'ROS_IP': '0.0.0.0'}],
        output='screen',
        condition=IfCondition(start_endpoint),
    )

    conformance = Node(
        package='simulation_service_tests',
        executable='service_conformance',
        name='service_conformance',
        output='screen',
        emulate_tty=True,
        arguments=['--profile', profile, '--junit', junit, '--no-color'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='diffbot',
                              description='テスト用ロボットのプロファイル'),
        DeclareLaunchArgument('junit', default_value='/tmp/service_conformance.xml',
                              description='JUnit XML の出力先'),
        DeclareLaunchArgument('start_endpoint', default_value='true',
                              description='ROS-TCP-Endpoint も起動するか'),
        endpoint,
        conformance,
    ])
