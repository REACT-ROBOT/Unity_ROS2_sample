#!/usr/bin/env python3
"""GNSS センサの結合テスト。

URDF の <simulation><sensor type="gnss"> で宣言した GNSS センサが、
スポーン後に /<robot>/<link>/fix (sensor_msgs/NavSatFix) を配信するかを、
動いているシミュレータへ接続して確認する。コンテナの中で、エンドポイントと
シミュレータを上げてから実行する:

    python3 scripts/gnss_sensor_test.py

シナリオ: 測地原点 (東京駅: 35.681236, 139.767125) を指定した箱をワールド
原点付近にスポーンして再生。ロボットはほぼ原点にいるので、緯度経度は指定した
原点にほぼ一致し、高度は有限値になるはず。status / covariance も既定値
(FIX / GPS / 共分散 UNKNOWN) を確認する。

注意: 測地原点はシーン全体で共有され、最初にスポーンしたロボットの指定が
優先される。先に別の原点で GNSS ロボットを出したままだとこのテストは
一致チェックに失敗するので、まっさらな状態で実行すること。

終了コード: 0 = すべて期待どおり / 1 = 不一致あり / 2 = 実行できなかった
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import ResetSimulation, SetSimulationState, SpawnEntity

RESULT_OK = 1
ALREADY_IN_TARGET_STATE = 101

ROBOT_NAME = 'gnss_test'
LINK_NAME = 'base_link'
ORIGIN_LAT = 35.681236
ORIGIN_LON = 139.767125
ORIGIN_ALT = 10.0
LATLON_TOL = 0.01  # deg (原点上のロボットなら実際は 1e-6 deg オーダー)

GNSS_URDF = f"""<?xml version="1.0"?>
<robot name="{ROBOT_NAME}">
  <link name="{LINK_NAME}">
    <inertial>
      <mass value="1.0"/>
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
    <sensor type="gnss" name="{LINK_NAME}">
      <update_rate>10</update_rate>
      <origin_latitude>{ORIGIN_LAT}</origin_latitude>
      <origin_longitude>{ORIGIN_LON}</origin_longitude>
      <origin_altitude>{ORIGIN_ALT}</origin_altitude>
    </sensor>
  </simulation>
</robot>
"""


class GnssProbe(Node):
    def __init__(self):
        super().__init__('gnss_sensor_probe')
        self.fixes = []  # NavSatFix メッセージそのもの
        self.create_subscription(
            NavSatFix, f'/{ROBOT_NAME}/{LINK_NAME}/fix', self._on_fix, 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_cli = self.create_client(SetSimulationState, '/set_simulation_state')
        self.reset_cli = self.create_client(ResetSimulation, '/reset_simulation')

    def _on_fix(self, msg):
        self.fixes.append(msg)

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
    probe = GnssProbe()
    checks = []

    def check(name, ok, detail):
        checks.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    try:
        # 箱をワールド原点付近 (z=0.1 m) にスポーン (resource_string 渡し、ファイル不要)
        req = SpawnEntity.Request()
        req.name = ROBOT_NAME
        req.allow_renaming = False
        req.entity_resource.uri = ''
        req.entity_resource.resource_string = GNSS_URDF
        req.entity_namespace = ''
        req.initial_pose.pose.position.z = 0.1
        res = probe.call(probe.spawn_cli, req)
        if res.result.result != RESULT_OK:
            print(f'FATAL: spawn failed: {res.result.result} "{res.result.error_message}"')
            return 2
        print(f'spawned {res.entity_name}')

        # 再生して測位を集める
        probe.set_state(SimulationState.STATE_PLAYING)
        probe.collect(4.0)

        check('fix topic published', len(probe.fixes) > 0,
              f'{len(probe.fixes)} msgs')

        if probe.fixes:
            last = probe.fixes[-1]

            # 原点上のロボットなので緯度経度は測地原点にほぼ一致するはず
            lat_err = abs(last.latitude - ORIGIN_LAT)
            lon_err = abs(last.longitude - ORIGIN_LON)
            check('latitude near origin', lat_err < LATLON_TOL,
                  f'lat = {last.latitude:.6f} (origin {ORIGIN_LAT}, err {lat_err:.2e} deg)')
            check('longitude near origin', lon_err < LATLON_TOL,
                  f'lon = {last.longitude:.6f} (origin {ORIGIN_LON}, err {lon_err:.2e} deg)')

            # 高度 = 原点高度 + Unity y。地面付近なので原点高度の近くの有限値
            alt_ok = math.isfinite(last.altitude) and abs(last.altitude - ORIGIN_ALT) < 10.0
            check('altitude finite and near origin altitude', alt_ok,
                  f'alt = {last.altitude:.3f} m (origin {ORIGIN_ALT} m)')

            # frame_id はセンサリンク名
            check('frame_id is sensor link', last.header.frame_id == LINK_NAME,
                  f'frame_id = "{last.header.frame_id}"')

            # シリアライザの既定値: STATUS_FIX / SERVICE_GPS / 共分散 UNKNOWN (9 要素の有限値)
            check('status is STATUS_FIX', last.status.status == NavSatStatus.STATUS_FIX,
                  f'status = {last.status.status}')
            check('service is SERVICE_GPS', last.status.service == NavSatStatus.SERVICE_GPS,
                  f'service = {last.status.service}')
            cov = list(last.position_covariance)
            cov_ok = (len(cov) == 9 and all(math.isfinite(c) for c in cov)
                      and last.position_covariance_type
                      == NavSatFix.COVARIANCE_TYPE_UNKNOWN)
            check('covariance sane (9 finite values, type UNKNOWN)', cov_ok,
                  f'type = {last.position_covariance_type}, cov[0] = {cov[0] if cov else "-"}')

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
