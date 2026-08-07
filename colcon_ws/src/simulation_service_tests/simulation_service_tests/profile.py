"""テスト対象ロボットのプロファイル (YAML から読む)。"""

import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory


class Profile:
    """1 台のテスト用ロボットと、その合否判定しきい値。"""

    def __init__(self, data):
        d = dict(data or {})

        self.name = d.get('name', 'unnamed')
        self.robot_name = d.get('robot_name', 'robot')

        # --- URDF の入手方法 ---------------------------------------------
        # xacro_package + xacro_file を指定すると実行時に展開して .urdf を作る。
        # urdf_path を直接指定してもよい。
        self.xacro_package = d.get('xacro_package')
        self.xacro_file = d.get('xacro_file')
        self.xacro_mappings = {str(k): str(v) for k, v in (d.get('xacro_mappings') or {}).items()}
        self.urdf_path = d.get('urdf_path')

        # --- サービス名 ---------------------------------------------------
        self.get_state_service = d.get('get_state_service', '/get_simulation_state')
        self.set_state_service = d.get('set_state_service', '/set_simulation_state')
        self.reset_service = d.get('reset_service', '/reset_simulation')
        self.spawn_service = d.get('spawn_service', '/spawn_entity')
        self.step_service = d.get('step_service', '/step_simulation')

        # --- トピック名 (未指定ならロボット名から組み立てる) ----------------
        self.joint_states_topic = d.get(
            'joint_states_topic', f'/{self.robot_name}/joint_states')
        self.joint_command_topic = d.get(
            'joint_command_topic', f'/{self.robot_name}/joint_command')
        self.ground_truth_topic = d.get('ground_truth_topic', '/ground_truth')

        # --- スポーン位置 (x, y, z, R, P, Y) -------------------------------
        self.spawn_pose = tuple(float(v) for v in d.get('spawn_pose', [0, 0, 0, 0, 0, 0]))

        # --- 指令方法と判定しきい値 ----------------------------------------
        self.command_mode = d.get('command_mode', 'position')  # position | velocity
        self.test_joints = d.get('test_joints') or []
        self.max_auto_joints = int(d.get('max_auto_joints', 2))
        self.command_amplitude = float(d.get('command_amplitude', 0.5))
        self.command_duration = float(d.get('command_duration', 3.0))
        self.absolute_limit = float(d.get('absolute_limit', 1.5))
        self.position_tolerance = float(d.get('position_tolerance', 0.15))
        self.motion_threshold = float(d.get('motion_threshold', 0.05))
        self.velocity_follow_ratio = float(d.get('velocity_follow_ratio', 0.3))

        # --- リセット判定しきい値 -----------------------------------------
        self.joint_reset_tolerance = float(d.get('joint_reset_tolerance', 0.1))
        self.pose_reset_tolerance = float(d.get('pose_reset_tolerance', 0.05))
        self.base_moves_under_command = bool(d.get('base_moves_under_command', False))

        # --- 待ち時間 -------------------------------------------------------
        self.spawn_settle_time = float(d.get('spawn_settle_time', 2.0))
        self.reset_settle_time = float(d.get('reset_settle_time', 1.0))
        self.topic_timeout = float(d.get('topic_timeout', 8.0))
        self.service_timeout = float(d.get('service_timeout', 15.0))
        self.reset_cycles = int(d.get('reset_cycles', 3))

    # ------------------------------------------------------------------
    def resolve_urdf(self, out_dir=None):
        """URDF の実ファイルパスを返す (必要なら xacro を展開する)。

        Unity 側は URI からファイルを読むので、シミュレータと同じファイル
        システム上に実体を置く必要がある。
        """
        if self.urdf_path:
            return os.path.abspath(os.path.expanduser(self.urdf_path))

        if not (self.xacro_package and self.xacro_file):
            raise ValueError(
                f"profile '{self.name}': urdf_path か xacro_package/xacro_file が必要")

        share = get_package_share_directory(self.xacro_package)
        xacro_path = os.path.join(share, self.xacro_file)
        if not os.path.exists(xacro_path):
            raise FileNotFoundError(xacro_path)

        out_dir = out_dir or tempfile.mkdtemp(prefix='sim_service_tests_')
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f'{self.robot_name}.urdf')

        cmd = ['xacro', xacro_path]
        cmd += [f'{k}:={v}' for k, v in self.xacro_mappings.items()]
        xml = subprocess.check_output(cmd, text=True)
        with open(out_path, 'w') as f:
            f.write(xml)
        return out_path


def load_profile(spec):
    """プロファイル名かファイルパスから Profile を作る。"""
    if os.path.exists(spec):
        path = spec
    else:
        share = get_package_share_directory('simulation_service_tests')
        path = os.path.join(share, 'config', f'{spec}.yaml')
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"プロファイル '{spec}' が見つからない (探した場所: {path})")
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    data.setdefault('name', os.path.splitext(os.path.basename(path))[0])
    return Profile(data)
