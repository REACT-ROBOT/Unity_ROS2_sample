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

from action_msgs.msg import GoalStatus
from simulation_interfaces.msg import Bounds, EntityCategory, Resource, SimulatorFeatures

from .sim_harness import (
    ALREADY_IN_TARGET_STATE,
    DEFAULT_SOURCES_FAILED,
    ENTITIES_SPAWN_FAILED,
    FEATURE_SERVICE_MAP,
    INCORRECT_TRANSITION,
    INVALID_POSE,
    NO_RESOURCE,
    NO_WORLD_LOADED,
    RESULT_FEATURE_UNSUPPORTED,
    RESULT_NOT_FOUND,
    RESULT_OK,
    RESULT_OPERATION_FAILED,
    ServiceTimeout,
    entity_result_name,
    result_name,
    spawn_result_name,
    state_name,
    world_result_name,
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
        self.features = set()         # G1 が読んだ get_simulator_features の申告
        self.entity_name = None       # H 群が使い回すエンティティ名

    def mark(self, flag):
        self.flags.add(flag)

    def has(self, flag):
        return flag in self.flags


def need_services(ctx, *names):
    """未実装のサービスがあれば SKIP の Outcome を返す。無ければ None。

    任意サービスを実装していないシミュレータでも、そのシナリオだけを
    SKIP にして残りは流せるようにするための共通処理。
    """
    missing = [n for n in names if not ctx.h.service_ready(n)]
    if missing:
        return skip(f'未実装: {", ".join(missing)}')
    return None


def ensure_entity(ctx, name=None):
    """エンティティが 1 体もなければスポーンして、その名前を返す。

    シナリオは順番に流れる前提だが、``--only`` で単体実行されることも
    あるので、必要な前提は自前で作れるようにしておく。
    """
    if ctx.h.service_ready('get_entities'):
        res = ctx.h.get_entities()
        if res.result.result == RESULT_OK and res.entities:
            return res.entities[0]
    wanted = name or ctx.p.robot_name
    res = ctx.h.spawn(ctx.urdf_path, name=wanted)
    if res.result.result != RESULT_OK:
        return None
    return res.entity_name or wanted


class Result:
    def __init__(self, scenario, status, detail, elapsed):
        self.scenario = scenario
        self.status = status
        self.detail = detail
        self.elapsed = elapsed


# ======================================================================
# A. 基本疎通
# ======================================================================

@scenario('A1', 'A. 基本疎通', '中核サービスがすべて discovery できる')
def a1_services_available(ctx):
    """任意サービスの有無はここでは見ない。

    エンティティ操作や world は simulation_interfaces でも任意扱いなので、
    揃っていないことを理由に A1 を落とすと以降が全部 SKIP になってしまう。
    実装と申告が食い違っていないかは G1 が、個々の挙動は H 群が確かめる。
    """
    missing = ctx.h.wait_for_services(ctx.p.service_timeout)
    if missing:
        return fail(f'見つからないサービス: {", ".join(missing)}')
    ctx.mark('services')

    optional = [name for name in FEATURE_SERVICE_MAP.values() if ctx.h.service_ready(name)]
    return ok('get/set_simulation_state, reset_simulation, spawn_entity, spawn_entities, '
              f'step_simulation, get_simulator_features を確認 '
              f'(任意サービスは {len(set(optional))} 件が応答)')


@scenario('A2', 'A. 基本疎通', '起動直後の状態は STOPPED', requires=('services',))
def a2_initial_state(ctx):
    state = ctx.h.current_state()
    if state == SimulationState.STATE_STOPPED:
        return ok('STOPPED')
    return fail(
        f'起動直後の状態が {state_name(state)}。'
        'SimulationState.msg では STOPPED が既定であるべき')


@scenario('A3', 'A. 基本疎通', 'result コードが Result.msg の規約に従う (RESULT_OK == 1)',
          requires=('services',))
def a3_result_code_convention(ctx):
    """かつては known_gap 宣言付きだった。

    「Unity 側が成功時に既定値 0 (= RESULT_FEATURE_UNSUPPORTED) を返す」は
    修正済みなので、宣言を外して回帰したら FAIL として出るようにしてある。
    """
    res = ctx.h.get_state()
    code = res.result.result
    if code == RESULT_OK:
        return ok('RESULT_OK(1) を返している')
    return fail(
        f'get_simulation_state が {result_name(code)} を返す。'
        'Result.msg では成功 = 1、0 は FEATURE_UNSUPPORTED を意味する')


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

@scenario('F1', 'F. 仕様適合', 'step_simulation が一時停止中に指定ステップだけ進める',
          requires=('services',))
def f1_step_simulation(ctx):
    """StepSimulation.srv の規定を実際に確かめる。

    以前は「未対応を明示的に返すこと」だけを見ていて、実装済みでも
    PASS になる書き方だった。実装された以上、進む量まで見る。

    刻み幅 (fixed timestep) はシミュレータ側の設定なのでテストからは
    分からない。そこで絶対値では判定せず、``n`` ステップと ``2n``
    ステップの進み量が 2 倍になることで比例関係を確かめる。
    """
    features = ctx.features or set(ctx.h.simulator_features().features.features)
    implemented = SimulatorFeatures.STEP_SIMULATION_SINGLE in features

    # まず「一時停止していないときは OPERATION_FAILED」(仕様どおり) を見る
    ctx.h.play()
    time.sleep(0.3)
    try:
        res = ctx.h.step(1)
    except ServiceTimeout as exc:
        return fail(f'step_simulation が無応答 ({exc})')

    if not implemented:
        code = res.result.result
        if code in (RESULT_OPERATION_FAILED, RESULT_FEATURE_UNSUPPORTED):
            return ok(f'未対応を明示している: {result_name(code)}: {res.result.error_message}')
        return fail(f'未対応と申告しているのに {result_name(code)} が返った')

    if res.result.result != RESULT_OPERATION_FAILED:
        return fail(
            f'PLAYING 中の step_simulation が {result_name(res.result.result)} を返した。'
            'StepSimulation.srv は「一時停止していなければ OPERATION_FAILED」と定めている')

    entity = ensure_entity(ctx)
    if entity is None:
        return skip('エンティティを用意できなかった')
    if not ctx.h.service_ready('get_entity_state'):
        # sim 時刻を停止中に読む手段が無い。停止したままであることだけ確かめる。
        ctx.h.pause()
        time.sleep(0.3)
        res = ctx.h.step(10)
        if res.result.result != RESULT_OK:
            return fail(f'一時停止中の step_simulation が {result_name(res.result.result)}')
        if ctx.h.current_state() != SimulationState.STATE_PAUSED:
            return fail('step_simulation の後に PAUSED へ戻っていない')
        return ok('進み量は get_entity_state が無いため未確認 (OK かつ PAUSED 維持は確認)')

    ctx.h.pause()
    time.sleep(0.3)

    def sim_time():
        res = ctx.h.get_entity_state(entity)
        if res.result.result != RESULT_OK:
            return None
        stamp = res.state.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    n = 25
    t0 = sim_time()
    if t0 is None:
        return skip(f"'{entity}' の状態を取得できなかった")
    first = ctx.h.step(n, timeout=max(ctx.p.service_timeout, 30.0))
    if first.result.result != RESULT_OK:
        return fail(f'一時停止中の step_simulation が {result_name(first.result.result)}: '
                    f'{first.result.error_message}')
    t1 = sim_time()
    second = ctx.h.step(2 * n, timeout=max(ctx.p.service_timeout, 60.0))
    if second.result.result != RESULT_OK:
        return fail(f'{2 * n} ステップの step_simulation が {result_name(second.result.result)}')
    t2 = sim_time()
    if t1 is None or t2 is None:
        return skip('ステップ後の sim 時刻を取得できなかった')

    d1, d2 = t1 - t0, t2 - t1
    if d1 <= 0:
        return fail(f'{n} ステップ進めたのに sim 時刻が進んでいない ({t0:.3f} -> {t1:.3f})')
    ratio = d2 / d1
    if not 1.7 <= ratio <= 2.3:
        return fail(
            f'進み量が比例しない: {n} ステップで {d1:.4f}s、{2 * n} ステップで {d2:.4f}s '
            f'(比 {ratio:.2f}、期待は約 2.0)')
    if ctx.h.current_state() != SimulationState.STATE_PAUSED:
        return fail('step_simulation の後に PAUSED へ戻っていない')

    return ok(f'{n} ステップ={d1:.4f}s、{2 * n} ステップ={d2:.4f}s (比 {ratio:.2f})、'
              f'1 ステップ≒{d1 / n * 1000:.1f}ms、終了後も PAUSED')


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

@scenario('G1', 'G. interfaces 2.x', 'get_simulator_features の申告がサービスの実体と一致する',
          requires=('services',))
def g1_simulator_features(ctx):
    """申告と実装のずれを両方向で検出する。

    以前はここに「未実装のはず」の機能名をベタ書きしていたが、それは
    ある時点のシミュレータの写しでしかなく、実装が進むたびにテスト側が
    嘘になった (実際 world 系と step_simulation の実装時に G1 だけが落ちた)。
    いまは ``FEATURE_SERVICE_MAP`` を使って

      * 申告しているのに ROS グラフにサービスが無い
      * サービスはあるのに申告していない

    の両方を突き合わせる。こうしておけば、対応表に載っている限り
    新しいサービスを実装してもテスト側の書き換えは要らない。
    """
    res = ctx.h.simulator_features()
    features = set(res.features.features)
    formats = list(res.features.spawn_formats)

    # 1 対 1 で対応するサービスが無い機能 (フラグだけのもの) は対象外。
    advertised_but_absent = []
    present_but_unadvertised = []
    for feature_name, service in FEATURE_SERVICE_MAP.items():
        code = getattr(SimulatorFeatures, feature_name, None)
        if code is None:
            continue  # simulation_interfaces が古く、この機能を知らない
        advertised = code in features
        ready = ctx.h.service_ready(service)
        if advertised and not ready:
            advertised_but_absent.append(f'{feature_name} ({service})')
        elif ready and not advertised:
            present_but_unadvertised.append(f'{feature_name} ({service})')

    if advertised_but_absent:
        return fail('申告されているのにサービスが見つからない: '
                    + ', '.join(sorted(advertised_but_absent)))
    if present_but_unadvertised:
        return fail('サービスはあるのに申告されていない: '
                    + ', '.join(sorted(present_but_unadvertised)))

    # 対応表に無い機能のうち、これだけは必ず申告されているべきもの
    required = {
        'SIMULATION_RESET_STATE': SimulatorFeatures.SIMULATION_RESET_STATE,
        'SIMULATION_RESET_SPAWNED': SimulatorFeatures.SIMULATION_RESET_SPAWNED,
    }
    missing = [name for name, code in required.items() if code not in features]
    if missing:
        return fail(f'実装済みなのに申告されていない機能: {", ".join(missing)}')

    if 'urdf' not in formats:
        return fail(f'spawn_formats に urdf が無い: {formats}')

    ctx.features = features
    ctx.mark('features')
    checked = len([n for n in FEATURE_SERVICE_MAP if getattr(SimulatorFeatures, n, None) is not None])
    return ok(f'{len(features)} 機能を申告 / サービスと突き合わせ {checked} 件が一致 / '
              f'spawn_formats={formats}')


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


@scenario('G6', 'G. interfaces 2.x', 'resource_string からスポーンできる (SPAWNING_RESOURCE_STRING)',
          requires=('services',))
def g6_spawn_from_resource_string(ctx):
    """uri を空にして定義そのものを文字列で渡す経路。

    mesh 参照が絡む場合、URDF がファイルとして存在しないので「URDF の隣」を
    起点にした解決ができない。検索パス (simulation_resources.json の
    spawnable_paths、および AMENT_PREFIX_PATH) から引けることまで見る。
    """
    features = ctx.features or set(ctx.h.simulator_features().features.features)
    supported = SimulatorFeatures.SPAWNING_RESOURCE_STRING in features

    with open(ctx.urdf_path, 'r') as f:
        urdf_text = f.read()

    resource = Resource()
    resource.uri = ''
    resource.resource_string = urdf_text

    ctx.h.stop()
    time.sleep(0.5)
    ctx.h.play()
    time.sleep(0.3)

    name = f'{ctx.p.robot_name}_from_string'
    res = ctx.h.spawn_raw(resource, name=name)
    code = res.result.result

    if not supported:
        if code == RESULT_OK:
            return fail('SPAWNING_RESOURCE_STRING を申告していないのに '
                        'resource_string でのスポーンが成功した')
        return ok(f'未対応を明示している: {spawn_result_name(code)}')

    if code != RESULT_OK:
        return fail(f'resource_string でのスポーンが {spawn_result_name(code)}: '
                    f'{res.result.error_message}')
    time.sleep(ctx.p.spawn_settle_time)

    if ctx.h.service_ready('get_entities'):
        listed = ctx.h.get_entities()
        if name not in listed.entities:
            return fail(f'スポーンしたはずの {name} が get_entities に出てこない: '
                        f'{list(listed.entities)}')

    # mesh を package:// で参照する定義を文字列で渡す。ファイルが無いので
    # 検索パスから引けなければ mesh が読めず、形の無いエンティティになる。
    mesh_note = ''
    mesh_urdf = (
        '<?xml version="1.0"?>\n'
        '<robot name="mesh_probe">\n'
        '  <link name="base_link">\n'
        '    <inertial><mass value="1.0"/>\n'
        '      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>\n'
        '    </inertial>\n'
        # STL は mm 単位なので、元の khr3 の URDF と同じ倍率を掛ける
        '    <visual><geometry>\n'
        '      <mesh filename="package://khr3_description/meshes/base_link.stl"'
        ' scale="0.001 0.001 0.001"/>\n'
        '    </geometry></visual>\n'
        '  </link>\n'
        '</robot>\n'
    )
    mesh_resource = Resource()
    mesh_resource.uri = ''
    mesh_resource.resource_string = mesh_urdf
    mesh_name = 'mesh_probe'
    mesh_res = ctx.h.spawn_raw(mesh_resource, name=mesh_name,
                               pose=(0.0, 4.0, 0.0, 0.0, 0.0, 0.0))
    if mesh_res.result.result != RESULT_OK:
        return fail(f'package:// mesh を含む resource_string のスポーンが '
                    f'{spawn_result_name(mesh_res.result.result)}: '
                    f'{mesh_res.result.error_message}')
    time.sleep(ctx.p.spawn_settle_time)

    if ctx.h.service_ready('get_entity_bounds'):
        eb = ctx.h.get_entity_bounds(mesh_name)
        if eb.result.result != RESULT_OK:
            return fail(f'get_entity_bounds({mesh_name}) が '
                        f'{result_name(eb.result.result)}')
        if eb.bounds.type != Bounds.TYPE_BOX or not eb.bounds.points:
            return fail(
                f'{mesh_name} に形が無い (bounds type={eb.bounds.type})。'
                'package:// の mesh が検索パスから解決できていない可能性が高い: '
                f'{eb.result.error_message}')
        hi, lo = eb.bounds.points[0], eb.bounds.points[1]
        size = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z)
        if min(size) <= 0.0:
            return fail(f'{mesh_name} の bounds が潰れている: {size}')
        # mm 単位の STL に 0.001 を掛けた大きさ。桁が違えば scale か
        # 読み込んだファイルそのものが想定と違う。
        if max(size) > 1.0:
            return fail(f'{mesh_name} の bounds が大きすぎる: {size}。'
                        'mesh の縮尺が想定と違う')
        mesh_note = (f' / package:// mesh も解決 '
                     f'({size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m)')

    ctx.h.delete_entity(mesh_name) if ctx.h.service_ready('delete_entity') else None

    return ok(f'{name} を定義文字列から生成{mesh_note}')


# ======================================================================
# H. エンティティ操作と world (simulation_interfaces の任意サービス)
# ======================================================================
#
# ここから下は「実装していなければ SKIP」で流す。任意サービスなので、
# 未実装のシミュレータを FAIL 扱いにはしない (申告と実体が食い違って
# いるかどうかは G1 が見ている)。

@scenario('H1', 'H. エンティティ操作', 'get_entities / get_entity_state が ground_truth と一致する',
          requires=('services',))
def h1_entity_state(ctx):
    unmet = need_services(ctx, 'get_entities', 'get_entity_state')
    if unmet:
        return unmet

    # 前段の名前空間テストなどが残っているので、まっさらから始める
    ctx.h.stop()
    time.sleep(0.5)
    ctx.h.play()
    time.sleep(0.3)
    name = f'{ctx.p.robot_name}_state'
    res = ctx.h.spawn(ctx.urdf_path, name=name)
    if res.result.result != RESULT_OK:
        return fail(f'スポーンが {spawn_result_name(res.result.result)}')
    time.sleep(ctx.p.spawn_settle_time)

    listed = ctx.h.get_entities()
    if listed.result.result != RESULT_OK:
        return fail(f'get_entities が {result_name(listed.result.result)}')
    if name not in listed.entities:
        return fail(f'get_entities に {name} が出てこない: {list(listed.entities)}')

    state = ctx.h.get_entity_state(name)
    if state.result.result != RESULT_OK:
        return fail(f'get_entity_state が {result_name(state.result.result)}')

    truth = ctx.h.base_pose(ctx.p.topic_timeout)
    if truth is None:
        return skip('ground_truth が来ないので突き合わせできない')

    p = state.state.pose.position
    dx = abs(p.x - truth[0])
    dy = abs(p.y - truth[1])
    dz = abs(p.z - truth[2])
    tol = max(ctx.p.pose_reset_tolerance, 0.05)
    if max(dx, dy, dz) > tol:
        return fail(
            f'get_entity_state の姿勢が ground_truth と食い違う: '
            f'service ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) vs '
            f'topic ({truth[0]:+.3f}, {truth[1]:+.3f}, {truth[2]:+.3f})。'
            'ROS (右手系 Z 上) と Unity (左手系 Y 上) の読み替えを疑うこと')

    frame = state.state.header.frame_id
    ctx.entity_name = name
    ctx.mark('entity_services')
    return ok(f'{name}: ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) が ground_truth と一致 '
              f'(frame_id={frame or "(空=world)"})')


@scenario('H2', 'H. エンティティ操作', 'set_entity_state で瞬間移動でき、不正な姿勢は拒否される',
          requires=('entity_services',))
def h2_set_entity_state(ctx):
    unmet = need_services(ctx, 'set_entity_state', 'get_entity_state')
    if unmet:
        return unmet

    name = ctx.entity_name
    target = (-2.0, 1.5, 0.0, 0.0, 0.0, 0.0)
    res = ctx.h.set_entity_state(name, pose=target)
    if res.result.result != RESULT_OK:
        return fail(f'set_entity_state が {entity_result_name(res.result.result)}: '
                    f'{res.result.error_message}')
    time.sleep(0.5)

    state = ctx.h.get_entity_state(name)
    p = state.state.pose.position
    dx, dy = abs(p.x - target[0]), abs(p.y - target[1])
    tol = max(ctx.p.pose_reset_tolerance, 0.05)
    if max(dx, dy) > tol:
        return fail(f'移動先が反映されない: 要求 ({target[0]:+.2f}, {target[1]:+.2f}) '
                    f'-> 実際 ({p.x:+.2f}, {p.y:+.2f})')

    # 正規化されていないクォータニオンは INVALID_POSE(101)
    bad = ctx.h.set_entity_state(name, orientation=(0.0, 0.0, 0.0, 0.0))
    if bad.result.result != INVALID_POSE:
        return fail(f'ゼロクォータニオンに対し {entity_result_name(bad.result.result)} が返った。'
                    'SetEntityState.srv では INVALID_POSE(101)')

    # 存在しないエンティティは NOT_FOUND(2)
    missing = ctx.h.set_entity_state('no_such_entity_probe', pose=target)
    if missing.result.result != RESULT_NOT_FOUND:
        return fail(f'存在しないエンティティに対し {entity_result_name(missing.result.result)} が返った。'
                    '期待値は NOT_FOUND(2)')

    return ok(f'({p.x:+.2f}, {p.y:+.2f}) へ移動、不正姿勢は INVALID_POSE(101)、'
              '未知の名前は NOT_FOUND(2)')


@scenario('H3', 'H. エンティティ操作', 'entity_info を書き戻せてタグで絞り込める',
          requires=('entity_services',))
def h3_entity_info(ctx):
    unmet = need_services(ctx, 'get_entity_info', 'set_entity_info', 'get_entities')
    if unmet:
        return unmet

    name = ctx.entity_name
    before = ctx.h.get_entity_info(name)
    if before.result.result != RESULT_OK:
        return fail(f'get_entity_info が {result_name(before.result.result)}')
    if before.info.category.category != EntityCategory.CATEGORY_ROBOT:
        return fail(
            f'URDF から作ったエンティティの既定カテゴリが '
            f'{before.info.category.category} (期待は CATEGORY_ROBOT=1)')

    tag = 'conformance_probe'
    res = ctx.h.set_entity_info(
        name, category=EntityCategory.CATEGORY_ROBOT, description='probe', tags=[tag])
    if res.result.result != RESULT_OK:
        return fail(f'set_entity_info が {result_name(res.result.result)}')

    after = ctx.h.get_entity_info(name)
    if list(after.info.tags) != [tag] or after.info.description != 'probe':
        return fail(f'書き戻した info が読み出せない: tags={list(after.info.tags)} '
                    f'description={after.info.description!r}')

    hit = ctx.h.get_entities(ctx.h.entity_filters(tags=[tag]))
    if name not in hit.entities:
        return fail(f'タグ {tag} で絞り込んでも {name} が出てこない: {list(hit.entities)}')
    miss = ctx.h.get_entities(ctx.h.entity_filters(tags=['no_such_tag_probe']))
    if miss.entities:
        return fail(f'一致しないタグで絞り込んだのに {list(miss.entities)} が返った')

    return ok(f'既定 CATEGORY_ROBOT、tags={[tag]} を書き戻して絞り込みも一致')


@scenario('H4', 'H. エンティティ操作', 'get_entity_bounds がロボットを包む箱を返す',
          requires=('entity_services',))
def h4_entity_bounds(ctx):
    unmet = need_services(ctx, 'get_entity_bounds')
    if unmet:
        return unmet

    res = ctx.h.get_entity_bounds(ctx.entity_name)
    if res.result.result != RESULT_OK:
        return fail(f'get_entity_bounds が {result_name(res.result.result)}: '
                    f'{res.result.error_message}')
    bounds = res.bounds
    if bounds.type != Bounds.TYPE_BOX:
        return fail(f'bounds.type が {bounds.type}。GetEntityBounds.srv は TYPE_BOX(1) を期待する')
    if len(bounds.points) != 2:
        return fail(f'TYPE_BOX の points が {len(bounds.points)} 個。2 個であるべき')

    hi, lo = bounds.points[0], bounds.points[1]
    size = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z)
    if min(size) <= 0.0:
        return fail(f'箱の辺が 0 以下: {size}。points は (upper right, lower left) の順')
    if max(size) > 100.0:
        return fail(f'箱が大きすぎる: {size}。単位が m でない可能性')

    return ok(f'{size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m の箱 (基準リンク座標系)')


@scenario('H5', 'H. エンティティ操作', 'EntityFilters の名前正規表現と bounds が効く',
          requires=('entity_services',))
def h5_entity_filters(ctx):
    unmet = need_services(ctx, 'get_entities', 'get_entity_state')
    if unmet:
        return unmet

    name = ctx.entity_name
    exact = ctx.h.get_entities(ctx.h.entity_filters(name_regex=name))
    if name not in exact.entities:
        return fail(f'完全一致の正規表現で {name} が出てこない: {list(exact.entities)}')
    nomatch = ctx.h.get_entities(ctx.h.entity_filters(name_regex='no_such_entity_probe'))
    if nomatch.entities:
        return fail(f'一致しない正規表現で {list(nomatch.entities)} が返った')

    # いまの位置を中心にした球なら必ず引っかかる / 遠くの球なら外れる
    state = ctx.h.get_entity_state(name)
    p = state.state.pose.position
    near = ctx.h.get_entities(
        ctx.h.entity_filters(bounds=ctx.h.sphere_bounds((p.x, p.y, p.z), 2.0)))
    if name not in near.entities:
        return fail(f'自分の位置を中心にした半径 2m の球で {name} が出てこない: '
                    f'{list(near.entities)}')
    far = ctx.h.get_entities(
        ctx.h.entity_filters(bounds=ctx.h.sphere_bounds((p.x + 500.0, p.y, p.z), 1.0)))
    if far.entities:
        return fail(f'500m 離れた球に {list(far.entities)} が引っかかった')

    # 凸包 (TYPE_CONVEX_HULL)
    features = ctx.features or set(ctx.h.simulator_features().features.features)
    supports_convex = SimulatorFeatures.ENTITY_BOUNDS_CONVEX in features
    here = (p.x, p.y, p.z)

    if not supports_convex:
        # 申告していないなら、黙って無視せず FEATURE_UNSUPPORTED を返すこと
        unsupported = ctx.h.get_entities(ctx.h.entity_filters(
            bounds=ctx.h.convex_bounds(ctx.h.tetrahedron_around(here, 4.0))))
        if unsupported.result.result != RESULT_FEATURE_UNSUPPORTED:
            return fail(
                f'ENTITY_BOUNDS_CONVEX を申告していないのに TYPE_CONVEX_HULL が '
                f'{result_name(unsupported.result.result)} を返した。'
                'EntityFilters は未対応の bounds 種別に FEATURE_UNSUPPORTED を返すと定めている')
        return ok('名前正規表現・球 bounds の内外・未対応 bounds の扱いを確認 '
                  '(凸包は未対応の申告どおり)')

    inside = ctx.h.get_entities(ctx.h.entity_filters(
        bounds=ctx.h.convex_bounds(ctx.h.tetrahedron_around(here, 4.0))))
    if inside.result.result != RESULT_OK:
        return fail(f'凸包での絞り込みが {result_name(inside.result.result)}: '
                    f'{inside.result.error_message}')
    if name not in inside.entities:
        return fail(f'自分を囲む四面体で {name} が出てこない: {list(inside.entities)}')

    far_center = (p.x + 500.0, p.y, p.z)
    outside = ctx.h.get_entities(ctx.h.entity_filters(
        bounds=ctx.h.convex_bounds(ctx.h.tetrahedron_around(far_center, 4.0))))
    if outside.entities:
        return fail(f'500m 離れた四面体に {list(outside.entities)} が引っかかった')

    # 同一平面上の点だけの凸包 (退化した入力)。エンティティを縦に貫く大きな四角形。
    # 高さは get_entity_bounds の中央に合わせて、確実に中を通るようにする。
    coplanar_note = ''
    if ctx.h.service_ready('get_entity_bounds'):
        eb = ctx.h.get_entity_bounds(name)
        if eb.result.result == RESULT_OK and eb.bounds.type == Bounds.TYPE_BOX:
            hi, lo = eb.bounds.points[0], eb.bounds.points[1]
            mid_y = p.y + (hi.y + lo.y) * 0.5
            mid_z = p.z + (hi.z + lo.z) * 0.5
            square = [
                (p.x - 5.0, mid_y, mid_z - 5.0),
                (p.x + 5.0, mid_y, mid_z - 5.0),
                (p.x + 5.0, mid_y, mid_z + 5.0),
                (p.x - 5.0, mid_y, mid_z + 5.0),
            ]
            flat = ctx.h.get_entities(ctx.h.entity_filters(
                bounds=ctx.h.convex_bounds(square)))
            if flat.result.result != RESULT_OK:
                return fail(f'同一平面上の凸包が {result_name(flat.result.result)}: '
                            f'{flat.result.error_message}')
            if name not in flat.entities:
                return fail(f'エンティティを貫く平面状の凸包で {name} が出てこない: '
                            f'{list(flat.entities)}。退化した凸包の扱いを疑うこと')
            coplanar_note = ' / 平面状の凸包も通る'

    # 点が足りない凸包は OPERATION_FAILED (Bounds.msg は 3 点以上と定めている)
    too_few = ctx.h.get_entities(ctx.h.entity_filters(
        bounds=ctx.h.convex_bounds([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])))
    if too_few.result.result == RESULT_OK:
        return fail('2 点しかない TYPE_CONVEX_HULL が成功扱いになった。'
                    'Bounds.msg は 3 点以上と定めている')

    return ok('名前正規表現・球 bounds の内外・凸包の内外'
              f'{coplanar_note} / 点不足の凸包は '
              f'{result_name(too_few.result.result)}')


@scenario('H6', 'H. エンティティ操作', 'delete_entity が指定した 1 体だけ消す',
          requires=('entity_services',))
def h6_delete_entity(ctx):
    unmet = need_services(ctx, 'delete_entity', 'get_entities')
    if unmet:
        return unmet

    keep = ctx.entity_name
    victim = f'{ctx.p.robot_name}_victim'
    res = ctx.h.spawn(ctx.urdf_path, name=victim, pose=(0.0, -2.5, 0.0, 0.0, 0.0, 0.0))
    if res.result.result != RESULT_OK:
        return fail(f'2 体目のスポーンが {spawn_result_name(res.result.result)}')
    time.sleep(ctx.p.spawn_settle_time)

    deleted = ctx.h.delete_entity(victim)
    if deleted.result.result != RESULT_OK:
        return fail(f'delete_entity が {result_name(deleted.result.result)}: '
                    f'{deleted.result.error_message}')

    again = ctx.h.delete_entity(victim)
    if again.result.result != RESULT_NOT_FOUND:
        return fail(f'消した後の delete_entity が {result_name(again.result.result)}。'
                    '期待値は NOT_FOUND(2)')

    remaining = ctx.h.get_entities()
    if victim in remaining.entities:
        return fail(f'削除したはずの {victim} が残っている: {list(remaining.entities)}')
    if keep not in remaining.entities:
        return fail(f'消していない {keep} まで消えた: {list(remaining.entities)}')

    return ok(f'{victim} だけ消え、{keep} は残った。再削除は NOT_FOUND(2)')


@scenario('H7', 'H. エンティティ操作', 'get_spawnables / 名前付き姿勢が応答する',
          requires=('services',))
def h7_resource_queries(ctx):
    unmet = need_services(ctx, 'get_spawnables', 'get_named_poses', 'get_named_pose_bounds')
    if unmet:
        return unmet

    spawnables = ctx.h.get_spawnables()
    if spawnables.result.result != RESULT_OK:
        return fail(f'get_spawnables が {result_name(spawnables.result.result)}: '
                    f'{spawnables.result.error_message}')

    poses = ctx.h.get_named_poses()
    if poses.result.result != RESULT_OK:
        return fail(f'get_named_poses が {result_name(poses.result.result)}: '
                    f'{poses.result.error_message}')

    # 存在しない名前は NOT_FOUND(2)
    missing = ctx.h.get_named_pose_bounds('no_such_named_pose_probe')
    if missing.result.result != RESULT_NOT_FOUND:
        return fail(f'未知の名前付き姿勢に対し {result_name(missing.result.result)} が返った。'
                    '期待値は NOT_FOUND(2)')

    if not poses.poses:
        # 探索先の設定は起動時にしか読めないので、設定が無いこと自体は失敗にしない
        return ok(f'spawnables {len(spawnables.spawnables)} 件 / 名前付き姿勢 0 件 '
                  '(simulation_resources.json が未設定。未知の名前が NOT_FOUND なのは確認)')

    first = poses.poses[0]
    bounds = ctx.h.get_named_pose_bounds(first.name)
    if bounds.result.result != RESULT_OK:
        return fail(f'get_named_pose_bounds({first.name}) が '
                    f'{result_name(bounds.result.result)}')

    # タグで絞り込めること (タグが付いている姿勢がある場合のみ)
    tagged = [p for p in poses.poses if p.tags]
    if tagged:
        tag = tagged[0].tags[0]
        hit = ctx.h.get_named_poses(tags=[tag])
        if not any(p.name == tagged[0].name for p in hit.poses):
            return fail(f'タグ {tag} で絞り込んでも {tagged[0].name} が出てこない')

    return ok(f'spawnables {len(spawnables.spawnables)} 件 / '
              f'名前付き姿勢 {len(poses.poses)} 件 (先頭 {first.name} の bounds も取得)')


@scenario('H8', 'H. world', 'world の読み込みと降ろしが状態と噛み合う', requires=('services',))
def h8_world_lifecycle(ctx):
    unmet = need_services(ctx, 'get_current_world', 'load_world', 'unload_world',
                          'get_available_worlds')
    if unmet:
        return unmet

    current = ctx.h.get_current_world()
    if current.result.result != RESULT_OK:
        return fail(f'起動時の get_current_world が '
                    f'{world_result_name(current.result.result)}。'
                    '起動直後は何らかのワールドが載っているはず')
    started_with = current.world.name

    listed = ctx.h.get_available_worlds()
    if listed.result.result not in (RESULT_OK, DEFAULT_SOURCES_FAILED):
        return fail(f'get_available_worlds が {world_result_name(listed.result.result)}')

    # 読めない world を渡しても、いま載っているものが壊れないこと
    broken = ctx.h.load_world(resource_string='{"not_a_scene": true}')
    if broken.result.result == RESULT_OK:
        return fail('シーンとして解釈できない resource_string が成功扱いになった')
    still = ctx.h.get_current_world()
    if still.result.result != RESULT_OK or still.world.name != started_with:
        return fail('読み込みに失敗した後にワールドが失われた。'
                    'load_world は消す前に検証すべき')

    # 降ろすと STATE_NO_WORLD になり、動かせなくなる
    unloaded = ctx.h.unload_world()
    if unloaded.result.result != RESULT_OK:
        return fail(f'unload_world が {world_result_name(unloaded.result.result)}')
    if ctx.h.current_state() != getattr(SimulationState, 'STATE_NO_WORLD', 4):
        return fail(f'unload_world 後の状態が {state_name(ctx.h.current_state())}。'
                    'SimulationState.msg では STATE_NO_WORLD(4)')
    again = ctx.h.unload_world()
    if again.result.result != NO_WORLD_LOADED:
        return fail(f'2 度目の unload_world が {world_result_name(again.result.result)}。'
                    '期待値は NO_WORLD_LOADED(101)')
    gone = ctx.h.get_current_world()
    if gone.result.result != NO_WORLD_LOADED:
        return fail(f'ワールドが無いのに get_current_world が '
                    f'{world_result_name(gone.result.result)}')
    blocked = ctx.h.play()
    if blocked.result.result == RESULT_OK:
        return fail('ワールドが無いのに PLAYING へ遷移できた。'
                    'STATE_NO_WORLD は「開始も停止も一時停止もできない」状態')

    # 読み込み直すと停止状態で戻ってくる
    restored = ctx.h.load_world(
        resource_string='{"objects":[{"type":"Cube","position":[0,0.5,0],'
                        '"rotationEuler":[0,0,0],"scale":[1,1,1],'
                        '"meshPath":"","isActive":true}]}',
        timeout=max(ctx.p.service_timeout, 40.0))
    if restored.result.result != RESULT_OK:
        return fail(f'resource_string からの load_world が '
                    f'{world_result_name(restored.result.result)}: '
                    f'{restored.result.error_message}')
    state = ctx.h.current_state()
    if state != SimulationState.STATE_STOPPED:
        return fail(f'load_world 後の状態が {state_name(state)}。'
                    'LoadWorld.srv は「読み込み後は停止状態」と定めている')

    # get_available_worlds が挙げた uri は実際に load_world へ渡せること。
    # 一覧と読み込みで受け付ける形式がずれていないかの確認。
    from_file = ''
    if listed.worlds:
        candidate = listed.worlds[0]
        uri = candidate.world_resource.uri
        if uri:
            loaded = ctx.h.load_world(uri=uri, timeout=max(ctx.p.service_timeout, 40.0))
            if loaded.result.result != RESULT_OK:
                return fail(
                    f'get_available_worlds が挙げた {uri} を load_world が受け付けない: '
                    f'{world_result_name(loaded.result.result)}: {loaded.result.error_message}')
            now = ctx.h.get_current_world()
            if now.world.name != candidate.name:
                return fail(f'{candidate.name} を読んだのに get_current_world が '
                            f'{now.world.name} を返す')
            from_file = f' / 一覧の {candidate.name} を uri で読み込み'

    return ok(f'起動時={started_with} / 一覧 {len(listed.worlds)} 件 / '
              f'不正な world で壊れない / 降ろすと NO_WORLD / 読み直すと STOPPED{from_file}')


@scenario('H9', 'H. world', 'world をタグで絞り込める (WORLD_TAGS)', requires=('services',))
def h9_world_tags(ctx):
    """WORLD_TAGS の基本動作と、異常な要求への応答をまとめて見る。

    タグ付きワールドはハーネスが用意する (``service_conformance_test.sh`` が
    indoor+warehouse / outdoor / タグ無し の 3 つを置く)。``--no-sim`` で
    相乗りした場合は用意されていないことがあるので、材料が足りなければ SKIP。
    """
    unmet = need_services(ctx, 'get_available_worlds')
    if unmet:
        return unmet

    features = ctx.features or set(ctx.h.simulator_features().features.features)
    supported = SimulatorFeatures.WORLD_TAGS in features

    if not supported:
        # 申告していないなら、黙って全部返すのではなく未対応と答えること
        res = ctx.h.get_available_worlds(tags=['indoor'])
        if res.result.result == RESULT_FEATURE_UNSUPPORTED:
            return ok('WORLD_TAGS 未対応を FEATURE_UNSUPPORTED(0) で明示している')
        return fail(
            f'WORLD_TAGS を申告していないのにタグ絞り込みが '
            f'{world_result_name(res.result.result)} を返した。'
            '未対応なら FEATURE_UNSUPPORTED(0) を返すべき')

    everything = ctx.h.get_available_worlds()
    if everything.result.result != RESULT_OK:
        return fail(f'絞り込み無しの get_available_worlds が '
                    f'{world_result_name(everything.result.result)}')
    by_name = {w.name: w for w in everything.worlds}
    tagged = {name: set(w.tags) for name, w in by_name.items() if w.tags}
    if not tagged:
        return skip('タグの付いたワールドが 1 つも無い '
                    '(--no-sim で相乗りした場合など。ハーネスが用意する 3 つが要る)')

    # 材料にする世界は「タグが最も多いもの」を選ぶ。名前順で選ぶと、たまたま
    # タグが 1 つの世界に当たったときに ALL の一致ケースを素通りしてしまう
    # (実際それで ALL の肯定側が一度も実行されていなかった)。
    # 同数のときは名前順にして、実行ごとに選択が変わらないようにする。
    sample_name, sample_tags = max(tagged.items(), key=lambda kv: (len(kv[1]), kv[0]))
    sample_tag = sorted(sample_tags)[0]
    expected_any = {n for n, t in tagged.items() if sample_tag in t}

    hit = ctx.h.get_available_worlds(tags=[sample_tag])
    if hit.result.result != RESULT_OK:
        return fail(f'タグ絞り込みが {world_result_name(hit.result.result)}')
    got = {w.name for w in hit.worlds}
    if got != expected_any:
        return fail(f'タグ {sample_tag} の絞り込み結果が合わない: '
                    f'期待 {sorted(expected_any)} / 実際 {sorted(got)}')
    untagged = [n for n, w in by_name.items() if not w.tags]
    if any(n in got for n in untagged):
        return fail(f'タグ指定なのにタグ無しのワールドが混ざった: {sorted(got)}')

    # タグは一覧の WorldResource にも載っていること
    if not by_name[sample_name].tags:
        return fail(f'{sample_name} の tags が空。一覧はタグを載せて返すべき')

    # ANY: 複数指定するとどれか 1 つでも一致したものが返る
    other = sorted({t for tags in tagged.values() for t in tags} - {sample_tag})
    detail_any = ''
    if other:
        both = ctx.h.get_available_worlds(tags=[sample_tag, other[0]], tags_mode=0)
        expected_both = {n for n, t in tagged.items() if sample_tag in t or other[0] in t}
        if {w.name for w in both.worlds} != expected_both:
            return fail(f'ANY で [{sample_tag}, {other[0]}] を指定した結果が合わない: '
                        f'期待 {sorted(expected_both)} / '
                        f'実際 {sorted(w.name for w in both.worlds)}')
        detail_any = f' / ANY[{sample_tag},{other[0]}]={len(expected_both)} 件'

    # ALL: 全部一致したものだけ返る
    detail_all = ''
    if len(sample_tags) >= 2:
        every = sorted(sample_tags)
        res = ctx.h.get_available_worlds(tags=every, tags_mode=1)
        expected_all = {n for n, t in tagged.items() if set(every) <= t}
        if {w.name for w in res.worlds} != expected_all:
            return fail(f'ALL で {every} を指定した結果が合わない: '
                        f'期待 {sorted(expected_all)} / '
                        f'実際 {sorted(w.name for w in res.worlds)}')
        detail_all = f' / ALL{every}={len(expected_all)} 件'
    if other:
        # 同時には成り立たない組み合わせ -> 0 件。ただしエラーではない
        impossible = ctx.h.get_available_worlds(tags=[sample_tag, other[0]], tags_mode=1)
        if impossible.result.result != RESULT_OK:
            return fail(f'ALL で一致しない組み合わせを指定したら '
                        f'{world_result_name(impossible.result.result)} が返った。'
                        '一致 0 件は失敗ではなく、空リストと OK であるべき')
        if impossible.worlds:
            return fail(f'ALL で両立しないタグを指定したのに '
                        f'{sorted(w.name for w in impossible.worlds)} が返った')

    # 存在しないタグ -> 0 件 + OK (見つからないことはエラーではない)
    unknown = ctx.h.get_available_worlds(tags=['no_such_world_tag_probe'])
    if unknown.result.result != RESULT_OK:
        return fail(f'存在しないタグで {world_result_name(unknown.result.result)} が返った。'
                    '一致 0 件は失敗ではない')
    if unknown.worlds:
        return fail(f'存在しないタグなのに {sorted(w.name for w in unknown.worlds)} が返った')

    # 未知の filter_mode -> 黙って ANY 扱いにせず失敗させること
    bogus = ctx.h.get_available_worlds(tags=[sample_tag], tags_mode=7)
    if bogus.result.result == RESULT_OK:
        return fail('未知の filter_mode=7 が成功扱いになった。'
                    '解釈できない絞り込みを黙って別の意味で実行してはいけない')

    # 読み込んだワールドのタグが get_current_world からも見えること
    detail_current = ''
    if ctx.h.service_ready('load_world') and ctx.h.service_ready('get_current_world'):
        uri = by_name[sample_name].world_resource.uri
        if uri:
            loaded = ctx.h.load_world(uri=uri, timeout=max(ctx.p.service_timeout, 40.0))
            if loaded.result.result != RESULT_OK:
                return fail(f'{sample_name} の load_world が '
                            f'{world_result_name(loaded.result.result)}')
            current = ctx.h.get_current_world()
            if set(current.world.tags) != sample_tags:
                return fail(
                    f'load_world 後の get_current_world のタグが '
                    f'{sorted(current.world.tags)}。期待は {sorted(sample_tags)}')
            detail_current = f' / 読み込み後も tags={sorted(sample_tags)}'

    return ok(f'一覧 {len(everything.worlds)} 件中 タグ {sample_tag} で '
              f'{len(expected_any)} 件{detail_any}{detail_all} / '
              f'未知タグは 0 件 OK / 未知 filter_mode は '
              f'{world_result_name(bogus.result.result)}{detail_current}')


# ======================================================================
# I. simulate_steps アクション
# ======================================================================

@scenario('I1', 'I. アクション', 'simulate_steps が feedback を返しながら指定ステップ進む',
          requires=('services',))
def i1_simulate_steps(ctx):
    """STEP_SIMULATION_ACTION の基本動作。

    サービス版 (F1) と違い、アクションは 1 ステップごとに feedback を返すと
    .action ファイルに書かれているので、その回数と中身まで見る。
    """
    features = ctx.features or set(ctx.h.simulator_features().features.features)
    supported = SimulatorFeatures.STEP_SIMULATION_ACTION in features
    ready = ctx.h.action_server_ready(timeout=5.0)

    if not supported:
        if ready:
            return fail('STEP_SIMULATION_ACTION を申告していないのに '
                        'simulate_steps アクションサーバが存在する')
        return skip('simulate_steps は未実装 (申告も無い)')
    if not ready:
        return fail('STEP_SIMULATION_ACTION を申告しているのに '
                    'simulate_steps アクションサーバが見つからない')

    entity = ensure_entity(ctx)
    if entity is None or not ctx.h.service_ready('get_entity_state'):
        return skip('sim 時刻を読むための get_entity_state / エンティティが無い')

    def sim_time():
        res = ctx.h.get_entity_state(entity)
        return None if res.result.result != RESULT_OK else (
            res.state.header.stamp.sec + res.state.header.stamp.nanosec * 1e-9)

    # 一時停止していないときは OPERATION_FAILED (.action の規定)
    ctx.h.play()
    time.sleep(0.3)
    status, result, _, accepted = ctx.h.simulate_steps(1)
    if not accepted:
        return fail('PLAYING 中のゴールが受理されなかった。'
                    'SimulateSteps.action は結果で OPERATION_FAILED を返すと定めている')
    if result.result.result != RESULT_OPERATION_FAILED:
        return fail(f'PLAYING 中の simulate_steps が '
                    f'{result_name(result.result.result)} を返した。期待は OPERATION_FAILED(4)')
    if status != GoalStatus.STATUS_ABORTED:
        return fail(f'失敗したゴールの status が {status}。期待は ABORTED(6)')

    # 一時停止して本番
    ctx.h.pause()
    time.sleep(0.3)
    steps = 20
    before = sim_time()
    status, result, feedback, accepted = ctx.h.simulate_steps(
        steps, timeout=max(ctx.p.service_timeout, 30.0))
    after = sim_time()

    if not accepted:
        return fail('一時停止中のゴールが受理されなかった')
    if result.result.result != RESULT_OK:
        return fail(f'simulate_steps が {result_name(result.result.result)}: '
                    f'{result.result.error_message}')
    if status != GoalStatus.STATUS_SUCCEEDED:
        return fail(f'完走したゴールの status が {status}。期待は SUCCEEDED(4)')
    # feedback の「数」も「最後の 1 件が届くこと」も、ROS のアクションでは約束できない。
    #
    #   * feedback は KEEP_LAST のトピックなので、読み側が追いつかなければ
    #     古いサンプルが上書きされて落ちる。
    #   * 終了直前に出した feedback は、結果 (サービス応答) に追い越されうる。
    #     rclpy のクライアントは結果を受け取った時点でその goal の feedback 購読を
    #     畳むので、追い越された分は捨てられる。jazzy の servo_demo では毎回
    #     末尾 2 件がこれで落ちた (20 件中 18 件、しかも落ちるのは必ず末尾)。
    #
    # 実際に何ステップ進んだかは下で sim 時刻から検証しているので、feedback には
    # 「経過が逐次報告されること」だけを求める。届いたものが 1..steps の狭義単調増加な
    # 部分列であること、および 1 件だけ (最後にまとめて 1 回) ではないことを見る。
    if not feedback:
        return fail('feedback が 1 件も来ていない')

    completed = [f.completed_steps for f in feedback]
    remaining = [f.remaining_steps for f in feedback]
    if any(c + r != steps for c, r in zip(completed, remaining)):
        return fail(f'completed_steps + remaining_steps が {steps} にならない: '
                    f'{list(zip(completed, remaining))}')
    if completed != sorted(set(completed)):
        return fail(f'completed_steps が狭義単調増加になっていない: {completed}')
    if not set(completed) <= set(range(1, steps + 1)):
        return fail(f'completed_steps に 1..{steps} の範囲外が混じっている: {completed}')
    if len(feedback) < 2:
        return fail('feedback が 1 件だけ。ステップごとの経過報告ではなく、'
                    '最後にまとめて 1 回出しているだけの可能性がある')
    if len(feedback) * 2 < steps:
        # 半分以上落ちるのは終了直前の追い越しでは説明がつかない
        return fail(f'feedback が {len(feedback)}/{steps} 件しか来ていない。'
                    '取りこぼしにしては少なすぎる')
    dropped = steps - len(feedback)
    drop_note = f' (末尾 {dropped} 件は結果に追い越されて欠落)' if dropped else ''

    if before is None or after is None:
        return skip('sim 時刻を取得できなかった')
    if after <= before:
        return fail(f'{steps} ステップ進めたのに sim 時刻が進んでいない '
                    f'({before:.3f} -> {after:.3f})')
    if ctx.h.current_state() != SimulationState.STATE_PAUSED:
        return fail('simulate_steps の後に PAUSED へ戻っていない')

    ctx.mark('simulate_steps')
    return ok(f'{steps} ステップで feedback {len(feedback)} 件{drop_note}、'
              f'sim 時刻 {before:.3f} -> {after:.3f} ({after - before:.3f}s)、終了後も PAUSED')


@scenario('I2', 'I. アクション', 'simulate_steps を途中でキャンセルできる',
          requires=('simulate_steps',))
def i2_simulate_steps_cancel(ctx):
    """キャンセル要求で早く終わり、status が CANCELED になること。

    ステップ数を多めに取っておいて、進みきる前にキャンセルを投げる。
    """
    ctx.h.pause()
    time.sleep(0.3)

    steps = 3000  # 50Hz なら 60s 相当。キャンセルしなければ終わらない長さ
    status, result, feedback, accepted = ctx.h.simulate_steps(
        steps, timeout=max(ctx.p.service_timeout, 30.0), cancel_after=1.0)

    if not accepted:
        return fail('ゴールが受理されなかった')
    if status != GoalStatus.STATUS_CANCELED:
        return fail(f'キャンセルしたゴールの status が {status}。期待は CANCELED(5)')
    if not feedback:
        return fail('キャンセルまでに feedback が 1 回も来ていない')

    done = feedback[-1].completed_steps
    if done >= steps:
        return fail(f'キャンセルしたのに {done}/{steps} ステップ完走している')
    if ctx.h.current_state() != SimulationState.STATE_PAUSED:
        return fail('キャンセル後に PAUSED へ戻っていない')

    # キャンセル後もサービスが生きていること (打ち切りで内部状態が壊れていない)
    follow_up = ctx.h.step(1)
    if follow_up.result.result != RESULT_OK:
        return fail(f'キャンセル直後の step_simulation が '
                    f'{result_name(follow_up.result.result)}。'
                    'キャンセルで stepping 状態が残っている可能性')

    return ok(f'{done}/{steps} ステップで打ち切り、status=CANCELED、'
              'その後の step_simulation も通る')


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
