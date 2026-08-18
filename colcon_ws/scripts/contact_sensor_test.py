#!/usr/bin/env python3
"""接触 (バンパー) センサの結合テスト。

URDF の <simulation><sensor type="contact"> で宣言した接触センサが、
スポーン後に /<robot>/<link>/contact (std_msgs/Bool) と
/<robot>/<link>/contact/wrench (geometry_msgs/WrenchStamped) を配信するかを、
動いているシミュレータへ接続して確認する。コンテナの中で、エンドポイントと
シミュレータを上げてから実行する:

    python3 scripts/contact_sensor_test.py

シナリオ: 1 kg の箱を空中 (z=0.5 m) にスポーンして再生。落下中は非接触、
地面に着いたら接触になり、静止後の正味接触力は約 m*g (9.81 N) になるはず。

終了コード: 0 = すべて期待どおり / 1 = 不一致あり / 2 = 実行できなかった
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import ResetSimulation, SetSimulationState, SpawnEntity

RESULT_OK = 1
ALREADY_IN_TARGET_STATE = 101

ROBOT_NAME = 'bumper_test'
LINK_NAME = 'base_link'
MASS = 1.0
GRAVITY = 9.81

BUMPER_URDF = f"""<?xml version="1.0"?>
<robot name="{ROBOT_NAME}">
  <link name="{LINK_NAME}">
    <inertial>
      <mass value="{MASS}"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <simulation>
    <sensor type="contact" name="{LINK_NAME}">
      <update_rate>20</update_rate>
    </sensor>
  </simulation>
</robot>
"""


class ContactProbe(Node):
    def __init__(self):
        super().__init__('contact_sensor_probe')
        self.bool_samples = []    # (wall, bool)
        self.wrench_samples = []  # (wall, force_z, |force|)
        self.create_subscription(
            Bool, f'/{ROBOT_NAME}/{LINK_NAME}/contact', self._on_bool, 10)
        self.create_subscription(
            WrenchStamped, f'/{ROBOT_NAME}/{LINK_NAME}/contact/wrench', self._on_wrench, 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_cli = self.create_client(SetSimulationState, '/set_simulation_state')
        self.reset_cli = self.create_client(ResetSimulation, '/reset_simulation')

    def _on_bool(self, msg):
        self.bool_samples.append((time.monotonic(), msg.data))

    def _on_wrench(self, msg):
        f = msg.wrench.force
        mag = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.wrench_samples.append((time.monotonic(), f.z, mag))

    def collect(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, timeout=15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                raise RuntimeError(f'service {client.srv_name} timed out')
        return future.result()

    def set_state(self, state):
        req = SetSimulationState.Request()
        req.state.state = state
        res = self.call(self.set_state_cli, req)
        if res.result.result not in (RESULT_OK, ALREADY_IN_TARGET_STATE):
            raise RuntimeError(
                f'set_simulation_state({state}) -> {res.result.result} '
                f'"{res.result.error_message}"')


def main():
    rclpy.init()
    probe = ContactProbe()
    checks = []

    def check(name, ok, detail):
        checks.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    try:
        # 箱を空中 z=0.5 m にスポーン (resource_string 渡し、ファイル不要)
        req = SpawnEntity.Request()
        req.name = ROBOT_NAME
        req.allow_renaming = False
        req.entity_resource.uri = ''
        req.entity_resource.resource_string = BUMPER_URDF
        req.entity_namespace = ''
        req.initial_pose.pose.position.z = 0.5
        res = probe.call(probe.spawn_cli, req)
        if res.result.result != RESULT_OK:
            print(f'FATAL: spawn failed: {res.result.result} "{res.result.error_message}"')
            return 2
        print(f'spawned {res.entity_name}')

        # 再生して落下・着地・静止まで待つ
        probe.set_state(SimulationState.STATE_PLAYING)
        probe.collect(4.0)

        check('bool topic published', len(probe.bool_samples) > 0,
              f'{len(probe.bool_samples)} msgs')
        check('wrench topic published', len(probe.wrench_samples) > 0,
              f'{len(probe.wrench_samples)} msgs')

        if probe.bool_samples:
            # 落下中の非接触 → 着地後の接触、の遷移が観測できること
            first_value = probe.bool_samples[0][1]
            saw_transition = any(
                (not a[1]) and b[1]
                for a, b in zip(probe.bool_samples, probe.bool_samples[1:]))
            landed = probe.bool_samples[-1][1]
            check('lands in contact', landed, f'last value = {landed}')
            check('falling starts without contact', (not first_value) or saw_transition,
                  f'first={first_value}, false->true transition seen={saw_transition}')

        if probe.wrench_samples:
            # 静止後の正味接触力は約 m*g、向きはセンサ系 +z (真上)。
            # かつて衝突ジオメトリが浮力用の ArticulationBody (既定質量 1 kg) を
            # 持っていて 2*m*g になっていたが、水要素を宣言しないロボットには
            # 付与しない修正が入り、いまは URDF 質量どおりになる。
            tail = probe.wrench_samples[-5:]
            avg_mag = sum(s[2] for s in tail) / len(tail)
            avg_z = sum(s[1] for s in tail) / len(tail)
            expected = MASS * GRAVITY
            check('resting force ~ m*g', expected * 0.8 <= avg_mag <= expected * 1.2,
                  f'|F| = {avg_mag:.2f} N (expected ~{expected:.2f} N)')
            check('force points up (+z in link frame)', avg_z > 0,
                  f'Fz = {avg_z:.2f} N')

        # 後始末: スポーン物を消し、停止に戻す
        probe.set_state(SimulationState.STATE_STOPPED)
        req = ResetSimulation.Request()
        req.scope = ResetSimulation.Request.SCOPE_SPAWNED
        probe.call(probe.reset_cli, req)

    except Exception as exc:  # noqa: BLE001 - 実行不能は 2 で報告する
        print(f'FATAL: {exc}')
        return 2
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    failed = [name for name, ok in checks if not ok]
    print()
    if failed:
        print(f'NG: {len(failed)}/{len(checks)} checks failed: {", ".join(failed)}')
        return 1
    print(f'OK: all {len(checks)} checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
