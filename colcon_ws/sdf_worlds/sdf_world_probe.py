#!/usr/bin/env python3
"""SDFワールド読み込みの実機検証プローブ。

動作中のシミュレータ+エンドポイントに対して:
 1. test_basic.world を load_world → get_current_world の名前と結果コード確認
 2. get_available_worlds に .world 2件が載るか (additional_sources 指定)
 3. diffbot をスポーンして PLAYING、/scan の幾何チェック
    - 0°: 壁 (box) 前面 x=2.9
    - -90°: 柱 (cylinder) 表面 1.7
 4. test_mesh.world を load_world
    - 0°: STLメッシュ壁前面 x=3.9 (向きが正しいときだけ当たる)
    - -90°: include した柱 1.7
 5. 後始末: unload_world → 組み込みシーンを resource_string で復元
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from simulation_interfaces.srv import (
    LoadWorld, UnloadWorld, GetCurrentWorld, GetAvailableWorlds,
    SpawnEntity, SetSimulationState, DeleteEntity)
from simulation_interfaces.msg import SimulationState
from sensor_msgs.msg import LaserScan

WORLD_DIR = "/home/unity/colcon_ws/sdf_worlds"
URDF = "/home/unity/colcon_ws/urdf_samples/diffbot.urdf"

FAILURES = []


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}: {label} {detail}")
    if not ok:
        FAILURES.append(label)


class Probe(Node):
    def __init__(self):
        super().__init__("sdf_world_probe")
        self.scan = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/diffbot/lidar_link/scan",
                                 self._on_scan, qos)

    def _on_scan(self, msg):
        self.scan = msg

    def call(self, srv_type, name, request, timeout=20.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service {name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"service {name} timed out")
        return future.result()

    def wait_scans(self, count=3, timeout=20.0):
        self.scan = None
        got = 0
        end = self.get_clock().now().nanoseconds / 1e9 + timeout
        while got < count:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.scan is not None:
                got += 1
                if got < count:
                    self.scan = None
            if self.get_clock().now().nanoseconds / 1e9 > end:
                return None
        return self.scan

    def range_at(self, scan, angle_deg):
        angle = math.radians(angle_deg)
        idx = round((angle - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(len(scan.ranges) - 1, idx))
        # ノイズに備えて近傍3本の中央値
        vals = sorted(scan.ranges[max(0, idx - 1):idx + 2])
        return vals[len(vals) // 2]


def load_world(node, uri):
    req = LoadWorld.Request()
    req.world_resource.uri = uri
    return node.call(LoadWorld, "/load_world", req, timeout=30.0)


def main():
    rclpy.init()
    node = Probe()

    # 1. test_basic.world
    res = load_world(node, f"file://{WORLD_DIR}/test_basic.world")
    check("load test_basic.world", res.result.result == 1,
          f"(result={res.result.result} msg='{res.result.error_message}')")
    check("world name from <world name=>", res.world.name == "test_basic",
          f"(got '{res.world.name}')")

    cur = node.call(GetCurrentWorld, "/get_current_world", GetCurrentWorld.Request())
    check("get_current_world", cur.world.name == "test_basic",
          f"(got '{cur.world.name}')")

    # 2. get_available_worlds
    req = GetAvailableWorlds.Request()
    req.additional_sources = [WORLD_DIR]
    req.continue_on_error = True
    avail = node.call(GetAvailableWorlds, "/get_available_worlds", req)
    names = sorted(w.name for w in avail.worlds)
    check("available worlds include SDF", "test_basic" in names and "test_mesh" in names,
          f"(got {names})")

    # 3. diffbot で幾何チェック
    sreq = SpawnEntity.Request()
    sreq.name = "diffbot"
    sreq.entity_resource.uri = f"file://{URDF}"
    sres = node.call(SpawnEntity, "/spawn_entity", sreq, timeout=60.0)
    check("spawn diffbot", sres.result.result == 1,
          f"(result={sres.result.result} '{sres.result.error_message}')")

    st = SetSimulationState.Request()
    st.state = SimulationState(state=SimulationState.STATE_PLAYING)
    node.call(SetSimulationState, "/set_simulation_state", st)

    scan = node.wait_scans()
    check("scan received (basic)", scan is not None)
    if scan is not None:
        front = node.range_at(scan, 0)
        right = node.range_at(scan, -90)
        check("box wall at 0 deg ~2.9", abs(front - 2.9) < 0.15, f"(got {front:.3f})")
        check("pillar at -90 deg ~1.7", abs(right - 1.7) < 0.15, f"(got {right:.3f})")

    # 4. test_mesh.world (load_world がエンティティも消す → 再スポーン)
    res = load_world(node, f"file://{WORLD_DIR}/test_mesh.world")
    check("load test_mesh.world", res.result.result == 1,
          f"(result={res.result.result} msg='{res.result.error_message}')")

    sres = node.call(SpawnEntity, "/spawn_entity", sreq, timeout=60.0)
    check("respawn diffbot", sres.result.result == 1)
    node.call(SetSimulationState, "/set_simulation_state", st)

    scan = node.wait_scans()
    check("scan received (mesh)", scan is not None)
    if scan is not None:
        front = node.range_at(scan, 0)
        right = node.range_at(scan, -90)
        oblique = node.range_at(scan, 20)  # 壁が直立していれば 3.9/cos20°=4.15
        check("mesh wall at 0 deg ~3.9", abs(front - 3.9) < 0.15, f"(got {front:.3f})")
        check("mesh wall at 20 deg ~4.15", abs(oblique - 4.15) < 0.2, f"(got {oblique:.3f})")
        check("included pillar at -90 deg ~1.7", abs(right - 1.7) < 0.15, f"(got {right:.3f})")

    # 5. 後始末: 組み込みシーンへ戻す
    node.call(UnloadWorld, "/unload_world", UnloadWorld.Request())
    req = LoadWorld.Request()
    req.world_resource.resource_string = '{"name":"SampleScene","objects":[]}'
    res = node.call(LoadWorld, "/load_world", req)
    check("restore built-in scene", res.result.result == 1)

    print("RESULT:", "ALL PASS" if not FAILURES else f"{len(FAILURES)} FAILURES: {FAILURES}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
