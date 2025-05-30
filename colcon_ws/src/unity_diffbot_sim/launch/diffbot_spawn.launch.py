import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import xacro

def generate_launch_description():
    diffbot_description_path = os.path.join(
        get_package_share_directory('diffbot_description'))

    xacro_file = os.path.join(diffbot_description_path,
                              'robots',
                              'diffbot.urdf.xacro')
    urdf_path = os.path.join(diffbot_description_path, 'robots', 'diffbot.urdf')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    relative_urdf_path = pathlib.Path(urdf_path).relative_to(os.getcwd())

    params = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("unity_diffbot_sim"),
            "config",
            "unity_diffbot.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
       remappings=[
            ('cmd_vel_stamped', '/diff_drive_controller/cmd_vel'),
        ],
    )
        
    spawn_robot = Node(
        package="simulation_ros2_utils",
        executable="spawn_entity",
        name="spawn_entity",
        output="screen",
        parameters=[{'urdf_path': urdf_path,
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0, #1.57,
                    }],
    )

    image_republish = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/diffbot/camera_link/image_raw'),
            ('out', '/camera_link/image_raw'),
        ],
        output='screen',
    )

    depth_image_republish = Node(
        package='image_transport',
        executable='republish',
        name='depth_image_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/diffbot/depth_camera_link/depth_image_raw'),
            ('out', '/depth_camera_link/depth_image_raw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        velocity_converter,
        spawn_robot,
        image_republish,
        depth_image_republish,
    ])
