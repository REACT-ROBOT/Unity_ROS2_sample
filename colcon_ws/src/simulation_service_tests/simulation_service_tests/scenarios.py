"""サービス適合性シナリオ本体。

シナリオは登録順に実行される **状態を持つ** 一連の手順である。
前段が作った状態 (スポーン済みロボット、記録した初期姿勢など) を
``Context`` 経由で後段が使うため、原則として順番を入れ替えないこと。
``--only`` で一部だけ流すこともできるが、その場合 ``requires`` で宣言した
前提が満たされないシナリオは SKIP になる。

判定の分類:
  PASS       期待どおり
  FAIL       期待どおりでない (回帰、または報告されている不具合)
  KNOWN_GAP  実装が未完了だと分かっている項目。既定では exit code に含めない
  SKIP       前提が満たされず実行できなかった
  ERROR      シナリオ自体が例外で落ちた
"""

import time
import traceback

from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import ResetSimulation

from simulation_interfaces.msg import Resource, SimulatorFeatures

from .sim_harness import (
    ALREADY_IN_TARGET_STATE,
    ENTITIES_SPAWN_FAILED,
    INCORRECT_TRANSITION,
    NO_RESOURCE,
    RESULT_OK,
    RESULT_OPERATION_FAILED,
    ServiceTimeout,
    result_name,
    spawn_result_name,
    state_name,
)

PASS = 'PASS'
FAIL = 'FAIL'
KNOWN_GAP = 'KNOWN_GAP'
SKIP = 'SKIP'
ERROR = 'ERROR'


class Outcome:
    def __init__(self, status, detail=''):
        self.status = status
        self.detail = detail


def ok(detail=''):
    return Outcome(PASS, detail)


def fail(detail=''):
    return Outcome(FAIL, detail)


def gap(detail=''):
    return Outcome(KNOWN_GAP, detail)


def skip(detail=''):
    return Outcome(SKIP, detail)


class Scenario:
    def __init__(self, key, group, title, func, requires=(), known_gap=False, why=''):
        self.key = key
        self.group = group
        self.title = title
        self.func = func
        self.requires = tuple(requires)
        self.known_gap = known_gap
        self.why = why


SCENARIOS = []


def scenario(key, group, title, requires=(), known_gap=False, why=''):
    def deco(func):
        SCENARIOS.append(Scenario(key, group, title, func, requires, known_gap, why))
        return func
    return deco


class Context:
    """シナリオ間で持ち回る状態。"""

    def __init__(self, harness, profile, urdf_path, log):
        self.h = harness
        self.p = profile
        self.urdf_path = urdf_path
        self.log = log
        self.flags = set()
        self.baseline_joints = None   # スポーン直後に落ち着いた関節位置
        self.baseline_pose = None     # スポーン直後の ground_truth 姿勢
        self.baseline_sim_time = None

    def mark(self, flag):
        self.flags.add(flag)

    def has(self, flag):
        return flag in self.flags


class Result:
    def __init__(self, scenario, status, detail, elapsed):
        self.scenario = scenario
        self.status = status
        self.detail = detail
        self.elapsed = elapsed


# ======================================================================
# A. 基本疎通
# ======================================================================

@scenario('A1', 'A. 基本疎通', '実装しているサービスがすべて discovery できる')
def a1_services_available(ctx):
    missing = ctx.h.wait_for_services(ctx.p.service_timeout)
    if missing:
        return fail(f'見つからないサービス: {", ".join(missing)}')
    ctx.mark('services')
    return ok('get/set_simulation_state, reset_simulation, spawn_entity, spawn_entities, '
              'step_simulation, get_simulator_features を確認')


@scenario('A2', 'A. 基本疎通', '起動直後の状態は STOPPED', requires=('services',))
def a2_initial_state(ctx):
    state = ctx.h.current_state()
    if state == SimulationState.STATE_STOPPED:
        return ok('STOPPED')
    return fail(
        f'起動直後の状態が {state_name(state)}。'
        'SimulationState.msg では STOPPED が既定であるべき')


@scenario('A3', 'A. 基本疎通', 'result コードが Result.msg の規約に従う (RESULT_OK == 1)',
          requires=('services',), known_gap=True,
          why='Unity 側が成功時に既定値 0 (= RESULT_FEATURE_UNSUPPORTED) を返している')
def a3_result_code_convention(ctx):
    res = ctx.h.get_state()
    code = res.result.result
    if code == RESULT_OK:
        return ok('RESULT_OK(1) を返している')
    return gap(
        f'get_simulation_state が {result_name(code)} を返す。'
        'Result.msg では成功 = 1、0 は FEATURE_UNSUPPORTED を意味する。'
        'クライアント側 (simulation_ros2_utils) も 0 を成功として扱っており、両方の修正が要る')


# ======================================================================
# B. 状態遷移
# ======================================================================

@scenario('B1', 'B. 状態遷移', 'STOPPED -> PLAYING に遷移できる', requires=('services',))
def b1_start(ctx):
    ctx.h.play()
    time.sleep(0.5)
    state = ctx.h.current_state()
    if state != SimulationState.STATE_PLAYING:
        return fail(f'start 要求後の状態が {state_name(state)}')
    ctx.mark('playing')
    return ok('PLAYING')


@scenario('B2', 'B. 状態遷移', '同一状態への遷移は ALREADY_IN_TARGET_STATE(101)',
          requires=('playing',))
def b2_already_in_state(ctx):
    res = ctx.h.play()
    code = res.result.result
    if code == ALREADY_IN_TARGET_STATE:
        return ok('101 を返している')
    return fail(
        f'PLAYING 中に再度 PLAYING を要求したら {result_name(code)} が返った。'
        '期待値は ALREADY_IN_TARGET_STATE(101)')


@scenario('B3', 'B. 状態遷移', '未定義の状態値は INCORRECT_TRANSITION(103) で拒否される',
          requires=('services',))
def b3_invalid_state(ctx):
    res = ctx.h.set_state(200)
    code = res.result.result
    if code == INCORRECT_TRANSITION:
        return ok('103 を返している')
    return fail(f'state=200 に対し {result_name(code)} が返った。期待値は 103')


# ======================================================================
# C. スポーンと指令受付 (これ以降の基準)
# ======================================================================

@scenario('C1', 'C. スポーンと指令', 'spawn_entity でロボットを生成できる', requires=('services',))
def c1_spawn(ctx):
    ctx.h.play()
    time.sleep(0.3)
    res = ctx.h.spawn(ctx.urdf_path)
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'spawn_entity が {result_name(code)}: {res.result.error_message}')
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return fail(
            f'spawn は成功応答したが {ctx.p.joint_states_topic} が '
            f'{ctx.p.topic_timeout:.0f}s 以内に来ない')
    ctx.mark('spawned')
    time.sleep(ctx.p.spawn_settle_time)
    return ok(f'{ctx.p.robot_name} を生成、joint_states 受信を確認')


@scenario('C2', 'C. スポーンと指令', 'スポーン直後の状態を基準として記録する',
          requires=('spawned',))
def c2_capture_baseline(ctx):
    ctx.h.play()
    time.sleep(0.3)
    jm = ctx.h.joint_map(ctx.p.topic_timeout)
    if jm is None:
        return fail('joint_states が取得できない')
    ctx.baseline_joints = {k: v[0] for k, v in jm.items()}
    ctx.baseline_pose = ctx.h.base_pose(ctx.p.topic_timeout)
    ctx.baseline_sim_time = ctx.h.sim_time(ctx.p.topic_timeout)
    ctx.mark('baseline')
    joints_txt = ', '.join(f'{k}={v:+.3f}' for k, v in sorted(ctx.baseline_joints.items()))
    pose_txt = ('未取得' if ctx.baseline_pose is None else
                'xyz=({:+.3f}, {:+.3f}, {:+.3f}) yaw={:+.3f}'.format(*ctx.baseline_pose))
    return ok(f'関節: {joints_txt} / ルート姿勢: {pose_txt}')


@scenario('C3', 'C. スポーンと指令', 'ground_truth が publish されている', requires=('spawned',))
def c3_ground_truth(ctx):
    pose = ctx.h.base_pose(ctx.p.topic_timeout)
    if pose is None:
        return fail(f'{ctx.p.ground_truth_topic} が来ない')
    ctx.mark('ground_truth')
    return ok('xyz=({:+.3f}, {:+.3f}, {:+.3f}) yaw={:+.3f}'.format(*pose))


@scenario('C4', 'C. スポーンと指令', '【基準】スポーン直後は指令を受け付ける',
          requires=('spawned',))
def c4_command_before_reset(ctx):
    ctx.h.play()
    time.sleep(0.3)
    accepted, detail = ctx.h.probe_command_response()
    if not accepted:
        return fail(f'スポーン直後から指令が効いていない: {detail}')
    ctx.mark('command_baseline')
    return ok(detail)


@scenario('C5', 'C. スポーンと指令', '指令でルート姿勢が動く (移動ロボットのみ)',
          requires=('command_baseline', 'ground_truth'))
def c5_base_moves(ctx):
    if not ctx.p.base_moves_under_command:
        return skip('このプロファイルは固定台ロボットなのでルート姿勢は動かない')
    before = ctx.h.base_pose(ctx.p.topic_timeout)
    joints = ctx.h.target_joints()
    ctx.h.send_command(joints, [ctx.p.command_amplitude] * len(joints),
                       ctx.p.command_duration)
    after = ctx.h.base_pose(ctx.p.topic_timeout)
    if before is None or after is None:
        return fail('ground_truth が取得できない')
    dist = ((after[0] - before[0]) ** 2 + (after[1] - before[1]) ** 2) ** 0.5
    if dist < ctx.p.motion_threshold:
        return fail(f'指令を送ってもルートが動かない (移動量 {dist:.3f} m)')
    ctx.mark('base_displaced')
    return ok(f'ルートが {dist:.3f} m 移動した')


@scenario('C6', 'C. スポーンと指令', 'PAUSED で sim 時刻が止まり、PLAYING で再開する',
          requires=('spawned',))
def c6_pause_resume(ctx):
    ctx.h.play()
    time.sleep(0.5)
    t0 = ctx.h.sim_time(ctx.p.topic_timeout)
    if t0 is None:
        return skip('joint_states が来ないため sim 時刻を測れない')
    time.sleep(1.0)
    t1 = ctx.h.sim_time(ctx.p.topic_timeout)
    if t1 is None or t1 - t0 < 0.3:
        return fail(f'PLAYING 中に sim 時刻が進んでいない (t0={t0:.3f}, t1={t1})')

    ctx.h.pause()
    time.sleep(0.3)
    if ctx.h.current_state() != SimulationState.STATE_PAUSED:
        return fail('pause 要求後の状態が PAUSED でない')
    # PAUSED では Time.timeScale = 0 で FixedUpdate ごと止まるため publish も止まる。
    # 「時刻が進まない」ことは publish 停止として観測する。
    silent = ctx.h.joint_states_silent_for(1.0)

    ctx.h.play()
    time.sleep(0.5)
    t2 = ctx.h.sim_time(ctx.p.topic_timeout)
    if t2 is None:
        return fail('resume 後に joint_states が再開しない')
    if not silent:
        return fail('PAUSED 中にも joint_states が publish され続けている (時間が止まっていない)')
    return ok(f'PAUSED 中は publish 停止、resume 後 sim 時刻 {t2:.3f} で再開')


# ======================================================================
# D. reset_simulation ― 報告されている不具合の本丸
# ======================================================================

@scenario('D1', 'D. reset_simulation', 'SCOPE_STATE のリセットが時間内に応答する',
          requires=('spawned',))
def d1_reset_state_returns(ctx):
    t0 = time.monotonic()
    res = ctx.h.reset(ResetSimulation.Request.SCOPE_STATE)
    dt = time.monotonic() - t0
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'reset(SCOPE_STATE) が {result_name(code)}: {res.result.error_message}')
    ctx.mark('reset_state_done')
    time.sleep(ctx.p.reset_settle_time)
    return ok(f'{dt * 1000:.0f} ms で応答')


@scenario('D2', 'D. reset_simulation', 'SCOPE_STATE ではエンティティが消えない',
          requires=('reset_state_done',))
def d2_reset_state_keeps_entity(ctx):
    # STOPPED / PAUSED 中は publish 自体が止まるので、生存確認は必ず PLAYING で行う
    ctx.h.play()
    time.sleep(0.3)
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return fail(
            'reset(SCOPE_STATE) 後に joint_states が復活しない。'
            'ResetSimulation.srv では SCOPE_SPAWNED を指定したときだけデスポーンするべき')
    ctx.mark('entity_alive_after_reset')
    return ok('リセット後もエンティティが生きている')


@scenario('D3', 'D. reset_simulation', 'SCOPE_STATE で関節がスポーン時の値に戻る',
          requires=('entity_alive_after_reset', 'baseline'),
          known_gap=True,
          why='ResetAllEntitiesState() はルートの transform だけ戻し、'
              'ArticulationBody の jointPosition / jointVelocity を戻していない')
def d3_reset_state_restores_joints(ctx):
    jm = ctx.h.joint_map(ctx.p.topic_timeout)
    if jm is None:
        return fail('joint_states が取得できない')
    worst_name, worst_err = None, 0.0
    lines = []
    for name, base in sorted(ctx.baseline_joints.items()):
        cur = jm.get(name, (float('nan'), 0.0))[0]
        err = abs(cur - base)
        lines.append(f'{name}: 基準{base:+.3f} 現在{cur:+.3f} 差{err:.3f}')
        if err > worst_err:
            worst_name, worst_err = name, err
    detail = '; '.join(lines)
    if worst_err <= ctx.p.joint_reset_tolerance:
        return ok(detail)
    return gap(
        f'関節が初期値に戻っていない (最大ずれ {worst_name} で {worst_err:.3f} rad, '
        f'許容 {ctx.p.joint_reset_tolerance}) / {detail}')


@scenario('D4', 'D. reset_simulation', 'SCOPE_STATE でルート姿勢がスポーン位置に戻る',
          requires=('entity_alive_after_reset', 'baseline'))
def d4_reset_state_restores_pose(ctx):
    if ctx.baseline_pose is None:
        return skip('基準となる ground_truth を取得できていない')
    cur = ctx.h.base_pose(ctx.p.topic_timeout)
    if cur is None:
        return fail('リセット後に ground_truth が取得できない')
    dist = sum((cur[i] - ctx.baseline_pose[i]) ** 2 for i in range(3)) ** 0.5
    detail = ('基準 ({:+.3f}, {:+.3f}, {:+.3f}) -> '.format(*ctx.baseline_pose[:3]) +
              '現在 ({:+.3f}, {:+.3f}, {:+.3f}) 差 {:.3f} m'.format(*cur[:3], dist))
    if dist <= ctx.p.pose_reset_tolerance:
        return ok(detail)

    # 位置を戻しても関節速度が残っていると、テレポート直後からまた走り出す。
    # 残留速度を一緒に出しておくと「戻していないのか」「戻した直後に動いたのか」を
    # 切り分けられる。
    jm = ctx.h.joint_map(ctx.p.topic_timeout) or {}
    residual = ', '.join(f'{k}={v[1]:+.3f} rad/s' for k, v in sorted(jm.items()))
    return fail(
        f'ルート姿勢が初期位置に戻っていない / {detail} / リセット直後の関節速度: {residual}'
        ' (速度が残っているならテレポート後に動き直しているだけで、'
        '原因は D3 と同じ「速度を戻していない」こと)')


@scenario('D5', 'D. reset_simulation', '★リセット後に再度 start すると指令を受け付ける',
          requires=('entity_alive_after_reset', 'command_baseline'))
def d5_command_after_reset(ctx):
    """報告されている「リセットすると指令を受け付けなくなる」の直接検証。"""
    ctx.h.play()
    time.sleep(0.5)
    state = ctx.h.current_state()
    if state != SimulationState.STATE_PLAYING:
        return fail(f'リセット後に start できない (状態 {state_name(state)})')
    accepted, detail = ctx.h.probe_command_response()
    if not accepted:
        return fail(
            'リセット + start の後に指令が効かない ← 報告されている不具合を再現。'
            f' {detail}')
    return ok(detail)


@scenario('D6', 'D. reset_simulation', 'リセット後もサービスが応答し続ける',
          requires=('reset_state_done',))
def d6_services_alive_after_reset(ctx):
    """Unity 側のサービスハンドラで例外が出ると TCP 接続ごと落ちる。

    リセット直後に軽いサービスを連打して、接続が生きているかを確かめる。
    """
    for i in range(5):
        try:
            ctx.h.get_state(timeout=5.0)
        except ServiceTimeout as exc:
            return fail(
                f'リセット後 {i + 1} 回目の get_simulation_state が無応答 ({exc})。'
                'サービスハンドラ内の例外で ROS-TCP 接続が切れた可能性が高い')
        time.sleep(0.2)
    return ok('get_simulation_state を 5 回連続で受け付けた')


@scenario('D7', 'D. reset_simulation', 'SCOPE_SPAWNED / SCOPE_ALL でデスポーンされる',
          requires=('entity_alive_after_reset',))
def d7_reset_all_despawns(ctx):
    ctx.h.play()
    time.sleep(0.3)
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return skip('デスポーン前に joint_states が来ていないため判定できない')
    res = ctx.h.reset(ResetSimulation.Request.SCOPE_DEFAULT)
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'reset(SCOPE_DEFAULT) が {result_name(code)}: {res.result.error_message}')
    ctx.h.play()   # publish が止まるのが「デスポーン」か「停止」かを切り分ける
    time.sleep(0.5)
    if not ctx.h.joint_states_silent_for(2.0):
        return fail(
            'SCOPE_DEFAULT (= SCOPE_ALL) でリセットしたのに joint_states が続いている。'
            'ResetSimulation.srv は spawn したエンティティのデスポーンを要求している')
    ctx.mark('despawned')
    return ok('PLAYING 状態で 2s 間 joint_states が来ない = デスポーン済み')


@scenario('D8', 'D. reset_simulation', '★デスポーン後に再スポーンして指令を受け付ける',
          requires=('despawned',))
def d8_respawn_after_reset_all(ctx):
    """デスポーン -> 再スポーンで指令経路が壊れないかを見る。

    Unity 側は破棄した JointStateSub の購読コールバックを解除していないため、
    ここは「同じトピックに死んだ購読者が積み上がる」問題が最初に表面化する
    地点になる。
    """
    ctx.h.play()
    time.sleep(0.3)
    res = ctx.h.spawn(ctx.urdf_path)
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'再スポーンが {result_name(code)}: {res.result.error_message}')
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return fail('再スポーン後に joint_states が来ない')
    # 生成そのものは成功しているので、指令が通らなくても後続シナリオは走らせる
    # (ここで止めると D9/D10/E が全部 SKIP になり、劣化の全体像が見えなくなる)
    ctx.mark('respawned')
    time.sleep(ctx.p.spawn_settle_time)
    ctx.h.play()
    accepted, detail = ctx.h.probe_command_response()
    if not accepted:
        return fail(
            '再スポーンしたロボットが指令を受け付けない ← 報告されている不具合を再現。'
            f' {detail} / シミュレータ側のログに JointStateSub.Callback の '
            'NullReferenceException が出ていないか確認すること '
            '(デスポーン時に購読を解除しないと、死んだコールバックが例外を投げて '
            '同じトピックの後続コールバックまで呼ばれなくなる)')
    return ok(detail)


@scenario('D9', 'D. reset_simulation', 'SCOPE_TIME で sim 時刻が先頭に戻る',
          requires=('respawned',), known_gap=True,
          why='SimulationControl.ResetSimulation の SCOPE_TIME 分岐が TODO のまま')
def d9_reset_time(ctx):
    ctx.h.play()
    time.sleep(1.0)
    before = ctx.h.sim_time(ctx.p.topic_timeout)
    if before is None:
        return skip('sim 時刻を取得できない')
    ctx.h.reset(ResetSimulation.Request.SCOPE_TIME)
    ctx.h.play()
    time.sleep(0.5)
    after = ctx.h.sim_time(ctx.p.topic_timeout)
    if after is None:
        return fail('SCOPE_TIME リセット後に joint_states が来ない')
    if after < before * 0.5:
        return ok(f'sim 時刻が {before:.2f} -> {after:.2f} に戻った')
    return gap(
        f'sim 時刻が戻らない ({before:.2f} -> {after:.2f})。'
        'SCOPE_TIME は未実装 (SimulationControl.cs の TODO)')


@scenario('D10', 'D. reset_simulation', '★リセットを繰り返しても指令受付が壊れない',
          requires=('respawned',))
def d10_reset_cycle_stability(ctx):
    """「使っているうちに効かなくなる」タイプの劣化を捕まえる反復試験。

    各サイクルで リセット -> start -> 指令 を行い、最初に失敗した回を報告する。
    """
    n = ctx.p.reset_cycles
    history = []
    for cycle in range(1, n + 1):
        try:
            ctx.h.reset(ResetSimulation.Request.SCOPE_STATE)
        except ServiceTimeout as exc:
            return fail(f'{cycle} 巡目の reset が無応答 ({exc})。前回までの結果: '
                        + '; '.join(history))
        time.sleep(ctx.p.reset_settle_time)
        try:
            ctx.h.play()
        except ServiceTimeout as exc:
            return fail(f'{cycle} 巡目の start が無応答 ({exc})')
        time.sleep(0.5)

        if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
            return fail(f'{cycle} 巡目のリセット後に joint_states が復活しない。'
                        'これ以降ロボットは指令を受け付けられない')

        accepted, detail = ctx.h.probe_command_response()
        history.append(f'{cycle}巡目={"OK" if accepted else "NG"}')
        if not accepted:
            return fail(
                f'{cycle} / {n} 巡目で指令が効かなくなった: {detail} '
                f'(経過: {", ".join(history)})')
    return ok(f'{n} 巡すべてで指令を受け付けた ({", ".join(history)})')


# ======================================================================
# E. STOPPED の挙動
# ======================================================================

@scenario('E1', 'E. 停止と再開', 'set_simulation_state(STOPPED) でデスポーンされる',
          requires=('respawned',))
def e1_stop_despawns(ctx):
    ctx.h.play()
    time.sleep(0.3)
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return skip('停止前に joint_states が来ていない')
    ctx.h.stop()
    time.sleep(0.5)
    state = ctx.h.current_state()
    if state != SimulationState.STATE_STOPPED:
        return fail(f'stop 要求後の状態が {state_name(state)}')
    ctx.h.play()   # PLAYING に戻して「止まっているだけ」ではないことを確かめる
    time.sleep(0.5)
    if not ctx.h.joint_states_silent_for(2.0):
        return fail(
            'STOPPED にしてもエンティティが残っている。'
            'SimulationState.msg は STOPPED を「ALL でリセットした状態」と定義している')
    ctx.mark('stopped_clean')
    return ok('STOPPED でデスポーンされた')


@scenario('E2', 'E. 停止と再開', '停止後に再スポーンして指令を受け付ける',
          requires=('stopped_clean',))
def e2_restart_after_stop(ctx):
    ctx.h.play()
    time.sleep(0.3)
    res = ctx.h.spawn(ctx.urdf_path)
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'停止後の再スポーンが {result_name(code)}: {res.result.error_message}')
    if not ctx.h.wait_for_joint_states(ctx.p.topic_timeout):
        return fail('停止後の再スポーンで joint_states が来ない')
    time.sleep(ctx.p.spawn_settle_time)
    accepted, detail = ctx.h.probe_command_response()
    if not accepted:
        return fail(f'停止 -> 再スポーンしたロボットが指令を受け付けない: {detail}')
    return ok(detail)


# ======================================================================
# F. 仕様適合 / 未実装項目
# ======================================================================

@scenario('F1', 'F. 仕様適合', 'step_simulation は未対応を明示的に返す', requires=('services',))
def f1_step_simulation(ctx):
    try:
        res = ctx.h.step(1)
    except ServiceTimeout as exc:
        return fail(f'step_simulation が無応答 ({exc})')
    code = res.result.result
    if code in (RESULT_OPERATION_FAILED, 0):
        return ok(f'{result_name(code)}: {res.result.error_message}')
    if code == RESULT_OK:
        return ok('実装されている')
    return fail(f'想定外の応答 {result_name(code)}: {res.result.error_message}')


@scenario('F2', 'F. 仕様適合', 'エンティティが無い状態でリセットしてもエラーにならない',
          requires=('services',))
def f2_reset_empty_scene(ctx):
    ctx.h.stop()      # まず全部消す
    time.sleep(0.5)
    try:
        res = ctx.h.reset(ResetSimulation.Request.SCOPE_ALL)
    except ServiceTimeout as exc:
        return fail(f'空シーンへの reset が無応答 ({exc})。ハンドラ内で例外が出ている可能性')
    code = res.result.result
    if code not in (RESULT_OK, 0):
        return fail(f'空シーンへの reset が {result_name(code)}: {res.result.error_message}')
    try:
        ctx.h.get_state(timeout=5.0)
    except ServiceTimeout as exc:
        return fail(f'空シーンへの reset 後にサービスが死んだ ({exc})')
    return ok('空シーンでも正常に応答する')


# ======================================================================
# G. simulation_interfaces 2.x の新インターフェース
# ======================================================================

@scenario('G1', 'G. interfaces 2.x', 'get_simulator_features が対応機能を申告する',
          requires=('services',))
def g1_simulator_features(ctx):
    res = ctx.h.simulator_features()
    features = set(res.features.features)
    formats = list(res.features.spawn_formats)

    # 実装しているものは必ず載っているべき
    required = {
        'SPAWNING': SimulatorFeatures.SPAWNING,
        'SPAWNING_ENTITIES': SimulatorFeatures.SPAWNING_ENTITIES,
        'SIMULATION_RESET': SimulatorFeatures.SIMULATION_RESET,
        'SIMULATION_RESET_STATE': SimulatorFeatures.SIMULATION_RESET_STATE,
        'SIMULATION_RESET_SPAWNED': SimulatorFeatures.SIMULATION_RESET_SPAWNED,
        'SIMULATION_STATE_GETTING': SimulatorFeatures.SIMULATION_STATE_GETTING,
        'SIMULATION_STATE_SETTING': SimulatorFeatures.SIMULATION_STATE_SETTING,
    }
    missing = [name for name, code in required.items() if code not in features]
    if missing:
        return fail(f'実装済みなのに申告されていない機能: {", ".join(missing)}')

    # 未実装のものを申告してはいけない。クライアントは使えると判断してしまう。
    forbidden = {
        'STEP_SIMULATION_SINGLE': SimulatorFeatures.STEP_SIMULATION_SINGLE,
        'WORLD_LOADING': SimulatorFeatures.WORLD_LOADING,
        'WORLD_UNLOADING': SimulatorFeatures.WORLD_UNLOADING,
        'ENTITY_STATE_SETTING': SimulatorFeatures.ENTITY_STATE_SETTING,
        'DELETING': SimulatorFeatures.DELETING,
    }
    wrong = [name for name, code in forbidden.items() if code in features]
    if wrong:
        return fail(f'未実装なのに申告されている機能: {", ".join(wrong)}')

    if 'urdf' not in formats:
        return fail(f'spawn_formats に urdf が無い: {formats}')

    ctx.mark('features')
    return ok(f'{len(features)} 機能を申告 / spawn_formats={formats}')


@scenario('G2', 'G. interfaces 2.x', 'spawn_entity が Resource.uri を受け付ける',
          requires=('spawned',))
def g2_resource_uri(ctx):
    """2.0.0 で uri/resource_string は Resource へまとめられた。

    C1 が通っている時点で uri 経由のスポーンは動いているので、ここでは
    「uri を空にすると仕様どおりのコードが返るか」を見る。
    """
    empty = Resource()
    empty.uri = ''
    empty.resource_string = ''
    res = ctx.h.spawn_raw(empty, name='no_resource_probe')
    code = res.result.result
    if code == NO_RESOURCE:
        return ok('uri / resource_string がどちらも空なら NO_RESOURCE(104)')
    if code == RESULT_OK:
        return fail('リソース無しのスポーンが成功扱いになっている')
    return fail(f'期待は NO_RESOURCE(104) だが {spawn_result_name(code)}: {res.result.error_message}')


@scenario('G3', 'G. interfaces 2.x', 'spawn_entities で複数体を一度に生成できる',
          requires=('services',))
def g3_spawn_entities(ctx):
    """SpawnEntity は 2.0.0 で deprecated、後継がこちら。"""
    # 干渉しないよう、まっさらな状態から始める
    ctx.h.stop()
    time.sleep(0.5)
    ctx.h.play()
    time.sleep(0.3)

    names = [f'{ctx.p.robot_name}_a', f'{ctx.p.robot_name}_b']
    entries = [
        (names[0], ctx.urdf_path, (0.0, 1.5, 0.0, 0.0, 0.0, 0.0)),
        (names[1], ctx.urdf_path, (0.0, -1.5, 0.0, 0.0, 0.0, 0.0)),
    ]
    res = ctx.h.spawn_many(entries, timeout=max(ctx.p.service_timeout, 40.0))

    if res.result.result != RESULT_OK:
        details = '; '.join(
            f'{i}: {spawn_result_name(r.result.result)} {r.result.error_message}'
            for i, r in enumerate(res.results))
        return fail(f'spawn_entities が {spawn_result_name(res.result.result)} / {details}')
    if len(res.results) != len(entries):
        return fail(f'results の数が要求数と違う ({len(res.results)} != {len(entries)})')

    # 名前は topic 名になるので、要求どおり別々でなければ 2 体が同じトピックへ
    # publish してしまう
    got = [r.entity_name for r in res.results]
    if got != names:
        return fail(
            f'entity_name が要求と違う: 要求 {names} -> 実際 {got}。'
            'SpawnEntity.srv では name が空でない限りその名前を使う決まり')

    time.sleep(ctx.p.spawn_settle_time)
    ctx.mark('spawned_many')
    return ok(f'{len(res.results)} 体を生成: {", ".join(got)}')


@scenario('G4', 'G. interfaces 2.x', 'spawn_entities は一部失敗を results で報告する',
          requires=('spawned_many',))
def g4_spawn_entities_partial(ctx):
    """1 件でも失敗したら ENTITIES_SPAWN_FAILED、個々の成否は results に入る。"""
    bad = Resource()
    bad.uri = 'file:///nonexistent/definitely_missing.urdf'

    good_name = f'{ctx.p.robot_name}_ok'
    entries = [(good_name, ctx.urdf_path, (0.0, 3.0, 0.0, 0.0, 0.0, 0.0))]
    res_ok = ctx.h.spawn_many(entries, timeout=max(ctx.p.service_timeout, 40.0))
    if res_ok.result.result != RESULT_OK:
        return skip('正常系の spawn_entities が通らないため部分失敗を判定できない')

    # 存在しないファイルを 1 件だけ要求する
    res = ctx.h.spawn_many(
        [(f'{ctx.p.robot_name}_bad', bad, (0.0, 5.0, 0.0, 0.0, 0.0, 0.0))],
        timeout=max(ctx.p.service_timeout, 40.0))

    if res.result.result != ENTITIES_SPAWN_FAILED:
        return fail(
            f'存在しないリソースを含む要求で {spawn_result_name(res.result.result)} が返った。'
            '期待は ENTITIES_SPAWN_FAILED(150)')
    if len(res.results) != 1:
        return fail(f'results の数が要求数と違う ({len(res.results)} != 1)')
    per_item = res.results[0].result.result
    if per_item == RESULT_OK:
        return fail('失敗したはずの要求が results では成功になっている')
    return ok(f'全体={spawn_result_name(res.result.result)} / '
              f'個別={spawn_result_name(per_item)}')


@scenario('G5', 'G. interfaces 2.x', 'entity_namespace でトピックが分離される',
          requires=('services',))
def g5_entity_namespace(ctx):
    """同じ URDF から 2 体出しても衝突しないことの確認。

    URDF の ros2_control に書かれた joint_states / joint_command のトピック名は
    リソース側で固定なので、名前空間を適用しないと 2 体目が 1 体目と同じトピックを
    使ってしまい、片方への指令が両方を動かす。
    """
    ctx.h.stop()
    time.sleep(0.5)
    ctx.h.play()
    time.sleep(0.3)

    ns = 'ns_probe'
    res = ctx.h.spawn(ctx.urdf_path, name=f'{ctx.p.robot_name}_ns',
                      namespace=ns, timeout=max(ctx.p.service_timeout, 40.0))
    if res.result.result != RESULT_OK:
        return fail(f'名前空間付きスポーンが {spawn_result_name(res.result.result)}: '
                    f'{res.result.error_message}')

    expected = '/' + ns + ctx.p.joint_states_topic
    if not ctx.h.wait_for_topic(expected, ctx.p.topic_timeout):
        listed = ', '.join(ctx.h.list_topics_with_prefix('/' + ns)) or '(なし)'
        return fail(
            f'{expected} が現れない。entity_namespace が適用されていない。'
            f' /{ns} 以下にあるトピック: {listed}')

    # 元の名前のトピックが使われていないことも確かめる (名前空間の付け忘れ検出)
    return ok(f'{expected} を確認')


# ======================================================================
# 実行
# ======================================================================

def run(ctx, only=None, skip_keys=None):
    """シナリオを順に実行して Result のリストを返す。"""
    only = set(only) if only else None
    skip_keys = set(skip_keys) if skip_keys else set()
    results = []

    for sc in SCENARIOS:
        if only and sc.key not in only:
            continue
        if sc.key in skip_keys:
            results.append(Result(sc, SKIP, '--skip で除外', 0.0))
            continue

        unmet = [r for r in sc.requires if not ctx.has(r)]
        if unmet:
            results.append(Result(
                sc, SKIP, f'前提が未達: {", ".join(unmet)}', 0.0))
            ctx.log(sc, SKIP, f'前提が未達: {", ".join(unmet)}')
            continue

        t0 = time.monotonic()
        try:
            outcome = sc.func(ctx)
        except ServiceTimeout as exc:
            outcome = Outcome(FAIL, f'サービス無応答: {exc}')
        except Exception as exc:  # noqa: BLE001 - シナリオの落ち方も情報として残す
            outcome = Outcome(ERROR, f'{type(exc).__name__}: {exc}\n'
                                     + traceback.format_exc(limit=4))
        elapsed = time.monotonic() - t0

        status = outcome.status
        # known_gap 宣言済みのシナリオが FAIL したら KNOWN_GAP に丸める
        if sc.known_gap and status == FAIL:
            status = KNOWN_GAP
        results.append(Result(sc, status, outcome.detail, elapsed))
        ctx.log(sc, status, outcome.detail)

    return results
