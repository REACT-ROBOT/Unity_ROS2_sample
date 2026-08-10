#!/usr/bin/env python3
"""/clock トピックの挙動検証。

Unity_ROS2_Robot_Simulator が publish する /clock (rosgraph_msgs/Clock) が
シミュレーション状態ごとに正しく振る舞うかを、動いているシミュレータへ
接続して確認する。コンテナの中で、エンドポイントとシミュレータを上げて
から実行する:

    python3 scripts/clock_topic_test.py

検証項目:
  1. /clock が publish されている (既定 100 Hz、実測 50 Hz 以上を要求)
  2. STOPPED / PAUSED 中も publish が続き、値は凍結されている
     (FixedUpdate 停止中も Update から実時間周期で流し続ける仕様の確認)
  3. PLAYING 中は sim 時刻が実時間とほぼ同じ速度で単調増加する
  4. reset_simulation SCOPE_TIME で値がほぼ 0 へ戻る

終了コード: 0 = すべて期待どおり / 1 = 不一致あり / 2 = 実行できなかった
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import GetSimulationState, ResetSimulation, SetSimulationState

RESULT_OK = 1
ALREADY_IN_TARGET_STATE = 101

# /clock は Update() 駆動なので、実レートは min(publishRate, フレームレート)。
# シミュレータは起動時に Application.targetFrameRate = 10 を設定する
# (FrameRateController、UI から変更可) ため、既定では ~10 Hz が設計どおりの値。
# その 8 割を下限とする。進行速度は WSL やコンテナ越しを考えて緩めに見る。
MIN_RATE_HZ = 8.0
FROZEN_TOLERANCE = 0.005    # 凍結中に許す値の揺れ [s]
ADVANCE_RATIO_RANGE = (0.5, 1.5)   # PLAYING 中の (sim 進み / 実時間) の許容範囲
RESET_MAX_VALUE = 1.0       # SCOPE_TIME 直後に許す sim 時刻 [s]


class ClockProbe(Node):
    def __init__(self):
        super().__init__('clock_topic_probe')
        self.samples = []  # (wall [monotonic], sim [s])
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.set_state_cli = self.create_client(SetSimulationState, '/set_simulation_state')
        self.get_state_cli = self.create_client(GetSimulationState, '/get_simulation_state')
        self.reset_cli = self.create_client(ResetSimulation, '/reset_simulation')

    def _on_clock(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.samples.append((time.monotonic(), sim))

    def collect(self, duration):
        """duration [s] の間 spin して、その間に届いた (wall, sim) を返す。"""
        self.samples = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.samples)

    def call(self, client, request, timeout=10.0):
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
                f'set_simulation_state({state}) -> result={res.result.result} '
                f'"{res.result.error_message}"')

    def reset_time(self):
        req = ResetSimulation.Request()
        req.scope = ResetSimulation.Request.SCOPE_TIME
        res = self.call(self.reset_cli, req)
        if res.result.result != RESULT_OK:
            raise RuntimeError(
                f'reset_simulation(SCOPE_TIME) -> result={res.result.result} '
                f'"{res.result.error_message}"')


def rate_of(samples):
    if len(samples) < 2:
        return 0.0
    span = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / span if span > 0 else 0.0


def sim_span(samples):
    values = [s for _, s in samples]
    return min(values), max(values)


def main():
    rclpy.init()
    probe = ClockProbe()
    checks = []

    def check(name, ok, detail):
        checks.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    try:
        # 0. /clock の存在確認
        print('waiting for /clock ...')
        deadline = time.monotonic() + 15.0
        while not probe.samples and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
        if not probe.samples:
            print('FATAL: /clock を 15 秒待っても受信できない。'
                  'シミュレータとエンドポイントは起動しているか?')
            return 2

        res = probe.call(probe.get_state_cli, GetSimulationState.Request())
        initial_state = res.state.state
        print(f'initial simulation state: {initial_state} (0=STOPPED)')

        # 1. 停止中: publish 継続 + 凍結
        probe.set_state(SimulationState.STATE_STOPPED)
        time.sleep(0.3)
        w = probe.collect(2.0)
        lo, hi = sim_span(w)
        check('stopped: publish rate', rate_of(w) >= MIN_RATE_HZ,
              f'{rate_of(w):.1f} Hz (>= {MIN_RATE_HZ})')
        check('stopped: value frozen', hi - lo <= FROZEN_TOLERANCE,
              f'spread {hi - lo:.6f} s @ {hi:.3f} s')

        # 2. 再生中: 実時間とほぼ同速で単調増加
        probe.set_state(SimulationState.STATE_PLAYING)
        time.sleep(0.3)
        w = probe.collect(3.0)
        wall = w[-1][0] - w[0][0]
        sim = w[-1][1] - w[0][1]
        ratio = sim / wall if wall > 0 else 0.0
        monotonic_ok = all(b[1] >= a[1] for a, b in zip(w, w[1:]))
        check('playing: publish rate', rate_of(w) >= MIN_RATE_HZ,
              f'{rate_of(w):.1f} Hz')
        check('playing: advances with real time',
              ADVANCE_RATIO_RANGE[0] <= ratio <= ADVANCE_RATIO_RANGE[1],
              f'sim {sim:.3f} s / wall {wall:.3f} s (ratio {ratio:.3f})')
        check('playing: monotonic', monotonic_ok, f'{len(w)} samples')
        value_before_pause = w[-1][1]

        # 3. 一時停止中: publish 継続 + 凍結 (Gazebo と同じ挙動)
        probe.set_state(SimulationState.STATE_PAUSED)
        time.sleep(0.3)
        w = probe.collect(2.0)
        lo, hi = sim_span(w)
        check('paused: publish rate', rate_of(w) >= MIN_RATE_HZ,
              f'{rate_of(w):.1f} Hz')
        check('paused: value frozen', hi - lo <= FROZEN_TOLERANCE,
              f'spread {hi - lo:.6f} s @ {hi:.3f} s')
        check('paused: value continues from playing', hi >= value_before_pause - 0.5,
              f'{hi:.3f} s (>= {value_before_pause - 0.5:.3f})')

        # 4. SCOPE_TIME リセットで 0 付近へ戻る
        probe.reset_time()
        time.sleep(0.3)
        w = probe.collect(1.0)
        _, hi_after = sim_span(w)
        check('reset SCOPE_TIME: value near zero', hi_after <= RESET_MAX_VALUE,
              f'{hi_after:.3f} s (<= {RESET_MAX_VALUE})')
        check('reset SCOPE_TIME: went backwards', hi_after < value_before_pause,
              f'{hi_after:.3f} s < {value_before_pause:.3f} s')

        # 5. リセット後も再生すればまた進む
        probe.set_state(SimulationState.STATE_PLAYING)
        time.sleep(0.3)
        w = probe.collect(2.0)
        sim = w[-1][1] - w[0][1]
        check('after reset: advances again', sim > 0.5, f'advanced {sim:.3f} s')

        # 後始末: 起動直後と同じ STOPPED に戻す
        probe.set_state(SimulationState.STATE_STOPPED)

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
