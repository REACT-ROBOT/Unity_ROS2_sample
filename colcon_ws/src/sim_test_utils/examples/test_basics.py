"""sim_test_utils の使用例 (実行にはシミュレータ + エンドポイントの起動が必要):

    cd ~/colcon_ws && python3 -m pytest src/sim_test_utils/examples -v
"""
import pytest

BOX_URDF = """<?xml version="1.0"?>
<robot name="testbox">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.015"/>
    </inertial>
    <visual><geometry><box size="0.3 0.3 0.3"/></geometry></visual>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
</robot>"""


def test_wrench_lifts_and_drops_box(sim):
    """上向き 30 N を 0.5 秒 → 1 kg の箱が跳び、切れると落ちて戻る。"""
    sim.spawn("pushbox", urdf=BOX_URDF, pose=(0.0, 0.0, 0.15))
    sim.play()
    sim.apply_wrench("pushbox", force=(0.0, 0.0, 30.0), duration=0.5)
    # 正味 (30 - 9.81) N / 1 kg で 0.5 s 加速 → 2 m 前後まで上がる
    sim.wait(lambda: sim.position("pushbox")[2] > 0.8,
             timeout=5.0, message="box lifted by wrench")
    sim.wait(lambda: sim.position("pushbox")[2] < 0.4,
             timeout=8.0, message="box fell back after wrench expired")


def test_torque_spins_box(sim):
    """ヨートルクだけを与えると、その場で回って位置は変わらない。"""
    sim.spawn("spinbox", urdf=BOX_URDF, pose=(1.0, 1.0, 0.5))
    sim.play()
    sim.apply_wrench("spinbox", torque=(0.0, 0.0, 2.0), duration=1.0)

    def spinning():
        s = sim.state("spinbox")
        return abs(s.twist.angular.z) > 1.0

    sim.wait(spinning, timeout=5.0, message="box spinning from torque")
    x, y, _ = sim.position("spinbox")
    assert abs(x - 1.0) < 0.2 and abs(y - 1.0) < 0.2, \
        "pure torque should not translate the box"


def test_stepping_is_deterministic(sim):
    """PAUSED + step_simulation で同じ初期状態から同じ軌道になる。

    PAUSED のままスポーン → ステップ → 消して再スポーン → 同じステップ、
    とすれば自由走行が一切混ざらず、開始状態が完全に一致する。
    """
    sim.play()
    sim.pause()

    def run(n_chunks=5, steps=10):
        sim.spawn("dropbox", urdf=BOX_URDF, pose=(0.0, 0.0, 1.0))
        zs = []
        for _ in range(n_chunks):
            sim.step(steps)
            zs.append(sim.position("dropbox")[2])
        sim.delete("dropbox")
        return zs

    first = run()
    second = run()

    assert first == pytest.approx(second, abs=1e-5), \
        f"stepped trajectories diverged: {first} vs {second}"


def test_wrench_rejects_unknown_entity(sim):
    from sim_test_utils import ServiceError
    with pytest.raises(ServiceError):
        sim.apply_wrench("no_such_entity", force=(1.0, 0.0, 0.0))
