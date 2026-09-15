"""シミュレータに prop (ロボット以外の置物) を 1 つ spawn する。

prop は URDF エンティティなので、get_entities に並び、delete_entity で消せ、
reset_simulation (SCOPE_SPAWNED) で片付く。

  ros2 launch sim_props_description spawn_prop.launch.py prop:=weeds
  ros2 launch sim_props_description spawn_prop.launch.py prop:=magnetic_course
  ros2 launch sim_props_description spawn_prop.launch.py prop:=gnss_canyon

prop は urdf/<prop>.urdf のファイル名。x/y/z/Y で置く場所をずらせる。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prop = LaunchConfiguration('prop')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('sim_props_description'), 'urdf', [prop, '.urdf'],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'prop', default_value='weeds',
            description='urdf/<prop>.urdf: weeds | magnetic_course | gnss_canyon'),
        DeclareLaunchArgument('name', default_value='',
                              description="エンティティ名。空なら URDF の robot 名"),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0', description='ヨー角 [rad]'),

        Node(
            package='simulation_ros2_utils',
            executable='spawn_entity',
            name='spawn_prop',
            output='screen',
            parameters=[{
                'urdf_path': ParameterValue(urdf_path, value_type=str),
                'robot_name': ParameterValue(LaunchConfiguration('name'), value_type=str),
                # spawn_entity 側は double で宣言しているので、置換の結果を
                # 文字列のまま渡すと型が合わずノードが落ちる。
                'x': ParameterValue(LaunchConfiguration('x'), value_type=float),
                'y': ParameterValue(LaunchConfiguration('y'), value_type=float),
                'z': ParameterValue(LaunchConfiguration('z'), value_type=float),
                'R': 0.0,
                'P': 0.0,
                'Y': ParameterValue(LaunchConfiguration('Y'), value_type=float),
            }],
        ),
    ])
