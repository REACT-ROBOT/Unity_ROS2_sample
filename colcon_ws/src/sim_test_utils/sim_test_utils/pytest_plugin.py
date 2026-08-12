"""pytest プラグイン: 起動済みのシミュレータ + エンドポイントに対して
テストシナリオを書くためのフィクスチャ。

    def test_something(sim):
        sim.spawn("box", urdf=..., pose=(0, 0, 1))
        sim.play()
        sim.apply_wrench("box", force=(0, 0, 30), duration=0.5)
        sim.wait(lambda: sim.position("box")[2] > 0.8)

セッションで rclpy を 1 回だけ初期化し、テストごとに
「スポーンしたエンティティの削除 + 停止 + 時刻リセット」を行う。
"""
import pytest

import rclpy

from .client import SimClient
from simulation_interfaces.srv import ResetSimulation


@pytest.fixture(scope="session")
def sim_session():
    rclpy.init()
    client = SimClient()
    yield client
    client.close()
    rclpy.shutdown()


@pytest.fixture
def sim(sim_session):
    yield sim_session
    # 後始末: テストが撒いたものを消し、止めて、時刻を戻す。
    sim_session.cleanup_spawned()
    try:
        sim_session.stop()
    except Exception:
        pass
    try:
        sim_session.reset(scope=ResetSimulation.Request.SCOPE_TIME)
    except Exception:
        pass
