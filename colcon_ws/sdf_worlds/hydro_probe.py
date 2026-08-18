#!/usr/bin/env python3
"""体積ベース浮力の実機検証プローブ。

phase "ground": 水なしで浮力宣言つき 1 kg 箱を接地させ、接地反力 ≈ 9.81 N を確認
                (旧実装はコリジョンごとに 1 kg のファントム体が付き 19.6 N になった)。
phase "water" : SIM_ENABLE_WATER=2 で起動したシミュレータに対し、
                比重 0.5 の箱を水面 (z=2) にスポーン → 中心が水面で釣り合うこと、
                比重 2.0 の箱が沈んで床に着くことを確認。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from simulation_interfaces.srv import SpawnEntity, SetSimulationState, GetEntityState, DeleteEntity
from simulation_interfaces.msg import SimulationState

FAILURES = []


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}: {label} {detail}")
    if not ok:
        FAILURES.append(label)


def box_urdf(name, material_density):
    return f"""<?xml version="1.0"?>
<robot name="{name}">
  <buoyancy_material name="mat">
    <density value="{material_density}"/>
  </buoyancy_material>
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.042" ixy="0" ixz="0" iyy="0.042" iyz="0" izz="0.042"/>
    </inertial>
    <visual>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
      <buoyancy_material name="mat"/>
    </collision>
  </link>
  <simulation>
    <sensor type="contact" name="base_link">
      <update_rate>20</update_rate>
    </sensor>
  </simulation>
</robot>"""


class Probe(Node):
    def __init__(self):
        super().__init__("hydro_probe")
        self.wrench = None
        self.create_subscription(WrenchStamped, "/buoybox/base_link/contact/wrench",
                                 self._on_wrench, 10)

    def _on_wrench(self, msg):
        self.wrench = msg

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"service {name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"service {name} timed out")
        return future.result()

    def spawn(self, name, urdf, z):
        req = SpawnEntity.Request()
        req.name = name
        req.entity_resource.resource_string = urdf
        req.initial_pose.pose.position.z = float(z)
        res = self.call(SpawnEntity, "/spawn_entity", req, timeout=60.0)
        return res.result.result == 1, res.result.error_message

    def play(self):
        req = SetSimulationState.Request()
        req.state = SimulationState(state=SimulationState.STATE_PLAYING)
        self.call(SetSimulationState, "/set_simulation_state", req)

    def entity_z(self, name):
        req = GetEntityState.Request()
        req.entity = name
        res = self.call(GetEntityState, "/get_entity_state", req)
        return res.state.pose.position.z

    def delete(self, name):
        req = DeleteEntity.Request()
        req.entity = name
        self.call(DeleteEntity, "/delete_entity", req)

    def wait_sim(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)


def phase_ground(node):
    ok, err = node.spawn("buoybox", box_urdf("buoybox", 2.0), 0.5)
    check("spawn buoyant box (ground phase)", ok, f"'{err}'")
    node.play()
    node.wait_sim(6.0)  # 落下・整定と接地反力の安定を待つ

    check("contact wrench received", node.wrench is not None)
    if node.wrench is not None:
        fz = node.wrench.wrench.force.z
        # リンク 1 kg のみ → 9.81 N。旧実装 (ファントム 1 kg) は 19.6 N
        check("normal force = m*g (no phantom mass)", abs(fz - 9.81) < 0.6,
              f"(got {fz:.2f} N, phantom-era value would be ~19.6 N)")


def phase_waterdbg(node):
    """浮き上がりの切り分け: zの時系列と、中性(1.0)・軽い(0.5)箱の比較。"""
    for name, dens in [("buoybox", 0.5), ("neutralbox", 1.0)]:
        ok, err = node.spawn(name, box_urdf(name, dens), 2.0)
        print(f"spawn {name} (density {dens}):", ok, err)
    node.play()
    for t in range(14):
        node.wait_sim(1.0)
        zs = {n: node.entity_z(n) for n in ("buoybox", "neutralbox")}
        print(f"t={t+1:2d}s  " + "  ".join(f"{n}: z={z:.3f}" for n, z in zs.items()))


def phase_water(node):
    # 比重 0.5: 水面 z=2 に置くと中心が水面で釣り合う。
    # シーンの水は波 (振幅 0.2) が立っているので、時間平均で判定する。
    ok, err = node.spawn("buoybox", box_urdf("buoybox", 0.5), 2.0)
    check("spawn light box (water phase)", ok, f"'{err}'")
    node.play()
    node.wait_sim(8.0)
    samples = []
    for _ in range(16):
        node.wait_sim(0.5)
        samples.append(node.entity_z("buoybox"))
    z_mean = sum(samples) / len(samples)
    z_span = max(samples) - min(samples)
    check("relative density 0.5 floats half submerged (mean z ~ 2.0, bobbing on waves)",
          abs(z_mean - 2.0) < 0.12,
          f"(mean z={z_mean:.3f}, bobbing span {z_span:.2f})")
    node.delete("buoybox")

    # 比重 2.0: 沈んで床 (z=0.25) に着く
    ok, err = node.spawn("sinkbox", box_urdf("sinkbox", 2.0), 2.0)
    check("spawn heavy box", ok, f"'{err}'")
    node.wait_sim(8.0)
    z_heavy = node.entity_z("sinkbox")
    check("relative density 2.0 sinks to the floor (z ~ 0.25)",
          abs(z_heavy - 0.25) < 0.1, f"(got z={z_heavy:.3f})")


def main():
    phase = sys.argv[1] if len(sys.argv) > 1 else "ground"
    rclpy.init()
    node = Probe()
    if phase == "ground":
        phase_ground(node)
    elif phase == "waterdbg":
        phase_waterdbg(node)
    else:
        phase_water(node)
    print("RESULT:", "ALL PASS" if not FAILURES else f"{len(FAILURES)} FAILURES: {FAILURES}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
