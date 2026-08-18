#!/usr/bin/env python3
"""effort コマンドモードの実機検証。

鉛直軸まわりに回る水平アーム (重力トルクなし) に定トルクを与え、
α = τ/I が成り立つかを /joint_states の速度系列の傾きで確かめる。
I = m L^2 + I_com = 1.0 * 0.5^2 + 0.001 = 0.251 kg m^2

checks:
 1. τ=0.5   → α ≈ 1.99 rad/s^2 (±7%)
 2. effort フィードバック ≈ 0.5
 3. τ=0     → ドライブが本当に無効なら ω はほぼ一定 (PD が生きていれば減速する)
 4. τ=100 (limit effort=10) → α ≈ 39.8 (クランプ確認)
 5. reset_simulation → ω=0 に戻り、保持トルクも破棄され再加速しない
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from simulation_interfaces.srv import SpawnEntity, SetSimulationState, ResetSimulation
from simulation_interfaces.msg import SimulationState

FAILURES = []
INERTIA = 0.251


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}: {label} {detail}")
    if not ok:
        FAILURES.append(label)


EFFORT_URDF = """<?xml version="1.0"?>
<robot name="effortbot">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <mass value="50.0"/>
      <inertia ixx="3.0" ixy="0" ixz="0" iyy="3.0" iyz="0" izz="3.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 0.8"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 0.8"/></geometry>
    </collision>
  </link>
  <joint name="pivot" type="continuous">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.85" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" velocity="100"/>
  </joint>
  <link name="arm_link">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.05"/></geometry>
    </visual>
    <collision>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.05"/></geometry>
    </collision>
  </link>
  <ros2_control name="effortbot" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">/effortbot/joint_command</param>
      <param name="joint_states_topic">/effortbot/joint_states</param>
    </hardware>
    <joint name="pivot">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
</robot>"""


class Probe(Node):
    def __init__(self):
        super().__init__("effort_probe")
        self.samples = []  # (stamp_sec, velocity, effort)
        self.create_subscription(JointState, "/effortbot/joint_states",
                                 self._on_state, 50)
        self.cmd_pub = self.create_publisher(JointState, "/effortbot/joint_command", 10)

    def _on_state(self, msg):
        if "pivot" not in msg.name:
            return
        i = msg.name.index("pivot")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
        eff = msg.effort[i] if i < len(msg.effort) else 0.0
        self.samples.append((stamp, vel, eff))

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"service {name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"service {name} timed out")
        return future.result()

    def command_effort(self, tau):
        msg = JointState()
        msg.name = ["pivot"]
        msg.effort = [float(tau)]
        self.cmd_pub.publish(msg)

    def wait_sim(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def clear(self):
        self.samples = []


def fit_alpha(samples):
    """(stamp, velocity) 系列に最小二乗で直線を当て、傾き α を返す。"""
    if len(samples) < 5:
        return None
    ts = [s[0] for s in samples]
    vs = [s[1] for s in samples]
    n = len(ts)
    tm = sum(ts) / n
    vm = sum(vs) / n
    denom = sum((t - tm) ** 2 for t in ts)
    if denom < 1e-9:
        return None
    return sum((t - tm) * (v - vm) for t, v in zip(ts, vs)) / denom


def main():
    rclpy.init()
    node = Probe()

    # 前回実行の残骸を消してから始める
    try:
        from simulation_interfaces.srv import DeleteEntity
        d = DeleteEntity.Request(); d.entity = "effortbot"
        node.call(DeleteEntity, "/delete_entity", d, timeout=10.0)
    except Exception:
        pass

    req = SpawnEntity.Request()
    req.name = "effortbot"
    req.entity_resource.resource_string = EFFORT_URDF
    res = node.call(SpawnEntity, "/spawn_entity", req, timeout=60.0)
    check("spawn effortbot", res.result.result == 1, f"'{res.result.error_message}'")

    st = SetSimulationState.Request()
    st.state = SimulationState(state=SimulationState.STATE_PLAYING)
    node.call(SetSimulationState, "/set_simulation_state", st)
    node.wait_sim(2.0)

    # 1. 定トルク 0.5 → α = τ/I
    node.clear()
    node.command_effort(0.5)
    node.wait_sim(3.0)
    alpha = fit_alpha(node.samples)
    expected = 0.5 / INERTIA
    check("constant torque: alpha = tau/I", alpha is not None and abs(alpha - expected) / expected < 0.07,
          f"(alpha={alpha if alpha is None else round(alpha, 3)}, expected {expected:.3f})")

    # 2. effort フィードバック
    efforts = [s[2] for s in node.samples[-10:]]
    eff_mean = sum(efforts) / len(efforts) if efforts else 0.0
    check("effort feedback ~ 0.5", abs(eff_mean - 0.5) < 0.05, f"(got {eff_mean:.3f})")

    # 3. トルクを 0 に → ドライブ無効なら ω はほぼ一定
    node.command_effort(0.0)
    node.wait_sim(1.0)  # 指令反映待ち
    node.clear()
    node.wait_sim(2.0)
    if len(node.samples) >= 5:
        v0 = node.samples[0][1]
        v1 = node.samples[-1][1]
        check("zero torque coasts (no hidden PD braking)",
              v0 > 1.0 and abs(v1 - v0) / abs(v0) < 0.05,
              f"(v {v0:.3f} -> {v1:.3f})")
    else:
        check("zero torque coasts (no hidden PD braking)", False, "(no samples)")

    # 4. リセットで止まり、保持トルクも破棄される
    reset = ResetSimulation.Request()
    reset.scope = ResetSimulation.Request.SCOPE_STATE
    node.call(ResetSimulation, "/reset_simulation", reset)
    node.call(SetSimulationState, "/set_simulation_state", st)  # リセット後は停止するので再度 PLAYING
    node.wait_sim(1.0)
    node.clear()
    node.wait_sim(2.0)
    vels = [abs(s[1]) for s in node.samples]
    check("reset clears held torque (stays still)",
          len(vels) > 0 and max(vels) < 0.05,
          f"(max |v| = {max(vels) if vels else -1:.4f})")

    # 5. クランプ: τ=100 だが limit effort=10。ω=0 から短い窓で測る
    # (長く回すと PhysX の関節速度上限に当たって傾きが鈍る)。
    node.clear()
    node.command_effort(100.0)
    node.wait_sim(0.8)
    # 指令がシミュレータへ届くまでの遅延中のサンプル (effort=0, ω=0) を
    # フィットに混ぜないよう、トルクが効いている区間だけを使う。
    active = [smp for smp in node.samples if smp[2] > 9.5]
    alpha = fit_alpha(active)
    expected = 10.0 / INERTIA
    check("torque clamped to <limit effort>", alpha is not None and abs(alpha - expected) / expected < 0.1,
          f"(alpha={alpha if alpha is None else round(alpha, 3)}, expected {expected:.3f})")
    node.command_effort(0.0)

    print("RESULT:", "ALL PASS" if not FAILURES else f"{len(FAILURES)} FAILURES: {FAILURES}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
