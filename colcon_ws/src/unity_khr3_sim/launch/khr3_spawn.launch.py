import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare

import xacro

def wrap_yaml_text(input_path: str, robot_name: str, output_path: str) -> None:
    is_joints_section = False
    # 元ファイルを行単位で読み込み
    with open(input_path, 'r') as fin:
        lines = fin.readlines()

    # 出力先を書き込みモードで開く
    with open(output_path, 'w') as fout:
        # 先頭にロボット名
        fout.write(f"{robot_name}:\n")
        # 元ファイルの各行を先頭２スペースでインデント
        for line in lines:
            # 空行はそのまま
            if line.strip() == "":
                fout.write("\n")
                is_joints_section = False
            elif line.startswith("    joints:"):
                is_joints_section = True
                fout.write(f"  {line}")
            else:
                if is_joints_section and line.startswith("      - "):
                    joint_name = line[len("      - "):].strip()
                    fout.write(f"        - {robot_name}_{joint_name}\n")
                else:
                    fout.write(f"  {line}")

def generate_launch_description():
    robot_name = 'KHR3_1'

    khr3_description_path = os.path.join(
        get_package_share_directory('khr3_description'))

    xacro_file = os.path.join(khr3_description_path,
                              'robots',
                              'khr3_001.urdf.xacro')
    urdf_path = os.path.join(khr3_description_path, 'robots', 'khr3_001.urdf')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true', 'robot_name': robot_name})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    params = {'robot_description': robot_desc}

    robot_description_path = os.path.join(
        get_package_share_directory('unity_khr3_sim'),
        'config',
        'unity_khr3.yaml'
    )
    tmp_robot_controllers_path = os.path.join(
        '/tmp',
        f'{robot_name}_sim.yaml'
    )
    wrap_yaml_text(robot_description_path, robot_name, tmp_robot_controllers_path)

    robot_controllers = PathJoinSubstitution(
        [
            tmp_robot_controllers_path,
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_robot = Node(
        package="simulation_ros2_utils",
        executable="spawn_entity",
        name="spawn_entity",
        output="screen",
        remappings=[
            ('spawn_entity', f'/spawn_entity'),
        ],
        parameters=[{'urdf_path': urdf_path,
                    'name': f"{robot_name}",
                    'x' : 0.0,
                    'y' : 0.3,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : -1.57,
                    }],
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", f"/{robot_name}/controller_manager"],
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", f"/{robot_name}/controller_manager"],
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(robot_name),

            node_robot_state_publisher,
            spawn_robot,
            control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
        ])
    ])