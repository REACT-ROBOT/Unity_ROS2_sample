import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node

import xacro


def generate_launch_description():
    robot_name = 'ServoDemo'

    description_path = get_package_share_directory('servo_demo_description')

    xacro_file = os.path.join(description_path, 'robots', 'servo_demo.urdf.xacro')
    urdf_path = os.path.join(description_path, 'robots', 'servo_demo.urdf')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true', 'robot_name': robot_name})
    robot_desc = doc.toprettyxml(indent='  ')
    with open(urdf_path, 'w') as f:
        f.write(robot_desc)

    params = {'robot_description': robot_desc}

    # 設定 yaml は名前空間 (ServoDemo) 込みで記述済み
    robot_controllers = os.path.join(description_path, 'config', 'unity_servo_demo.yaml')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_robot = Node(
        package='simulation_ros2_utils',
        executable='spawn_entity',
        name='spawn_entity',
        output='screen',
        remappings=[
            ('spawn_entity', '/spawn_entity'),
        ],
        parameters=[{'urdf_path': urdf_path,
                     'name': robot_name,
                     'x': 0.0,
                     'y': 0.0,
                     'z': 0.0,
                     'R': 0.0,
                     'P': 0.0,
                     'Y': -1.57,
                     }],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[params, robot_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # respawn: スポナーはシミュレータ側の初期化タイミング次第で失敗することが
    # あるため、成功するまでリトライさせる
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', f'/{robot_name}/controller_manager'],
        respawn=True,
        respawn_delay=5.0,
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', f'/{robot_name}/controller_manager'],
        respawn=True,
        respawn_delay=5.0,
    )

    # ステップ/低速スイープ指令を2関節へ同時に送り続けるデモノード
    commander = Node(
        package='servo_demo_description',
        executable='servo_demo_commander.py',
        name='servo_demo_commander',
        output='screen',
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(robot_name),

            node_robot_state_publisher,
            spawn_robot,
            control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            commander,
        ])
    ])
