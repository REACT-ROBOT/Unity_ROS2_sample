"""シミュレータへ接続してサービス/トピックを叩くための下回り。

重要な前提 (Unity 側実装に由来):

* ``Time.timeScale == 0`` の間 Unity の ``FixedUpdate`` は回らないため、
  STOPPED / PAUSED 中は ``joint_states`` も ``ground_truth`` も publish が
  完全に止まる。「エンティティが生きているか」をトピックの有無で判定する
  ときは必ず PLAYING にしてから観測すること。
* ``joint_states`` のタイムスタンプは Unity の ``Time.timeAsDouble``
  (= sim 時刻) なので、タイムアウト計算には使えない。本モジュールは一貫して
  ``time.monotonic()`` (壁時計) を使う。
"""

import math
import os
import threading
import time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3 as Vector3Msg
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from simulation_interfaces.msg import (
    Bounds,
    EntityCategory,
    EntityFilters,
    EntityInfo,
    Resource,
    SimulationState,
    TagsFilter,
)
from simulation_interfaces.msg import SpawnEntity as SpawnEntityMsg
from simulation_interfaces.srv import (
    DeleteEntity,
    GetAvailableWorlds,
    GetCurrentWorld,
    GetEntities,
    GetEntitiesStates,
    GetEntityBounds,
    GetEntityInfo,
    GetEntityState,
    GetNamedPoseBounds,
    GetNamedPoses,
    GetSimulationState,
    GetSimulatorFeatures,
    GetSpawnables,
    LoadWorld,
    ResetSimulation,
    SetEntityInfo,
    SetEntityState,
    SetSimulationState,
    SpawnEntities,
    SpawnEntity,
    StepSimulation,
    UnloadWorld,
)

# simulation_interfaces/msg/Result.msg
RESULT_FEATURE_UNSUPPORTED = 0
RESULT_OK = 1
RESULT_NOT_FOUND = 2
RESULT_INCORRECT_STATE = 3
RESULT_OPERATION_FAILED = 4

# SetSimulationState 固有の拡張コード
ALREADY_IN_TARGET_STATE = 101
STATE_TRANSITION_ERROR = 102
INCORRECT_TRANSITION = 103

# SpawnEntity / SpawnResult 固有の拡張コード (simulation_interfaces 2.x)
NAME_NOT_UNIQUE = 101
UNSUPPORTED_FORMAT = 103
NO_RESOURCE = 104
RESOURCE_PARSE_ERROR = 106
MISSING_ASSETS = 107

# SpawnEntities 固有の拡張コード
ENTITIES_SPAWN_FAILED = 150

# SetEntityState 固有の拡張コード
INVALID_POSE = 101

# LoadWorld 固有の拡張コード
WORLD_UNSUPPORTED_FORMAT = 101
WORLD_NO_RESOURCE = 102
WORLD_RESOURCE_PARSE_ERROR = 103
WORLD_MISSING_ASSETS = 104
WORLD_UNSUPPORTED_ASSETS = 105
WORLD_UNSUPPORTED_ELEMENTS = 106

# UnloadWorld / GetCurrentWorld 固有の拡張コード
NO_WORLD_LOADED = 101

# GetAvailableWorlds 固有の拡張コード
DEFAULT_SOURCES_FAILED = 101

STATE_NAMES = {
    SimulationState.STATE_STOPPED: 'STOPPED',
    SimulationState.STATE_PLAYING: 'PLAYING',
    SimulationState.STATE_PAUSED: 'PAUSED',
    SimulationState.STATE_QUITTING: 'QUITTING',
    # 2.0.0 で追加。ワールドを降ろしている間の状態。
    getattr(SimulationState, 'STATE_NO_WORLD', 4): 'NO_WORLD',
    getattr(SimulationState, 'STATE_LOADING_WORLD', 5): 'LOADING_WORLD',
}

RESULT_NAMES = {
    RESULT_FEATURE_UNSUPPORTED: 'FEATURE_UNSUPPORTED(0)',
    RESULT_OK: 'OK(1)',
    RESULT_NOT_FOUND: 'NOT_FOUND(2)',
    RESULT_INCORRECT_STATE: 'INCORRECT_STATE(3)',
    RESULT_OPERATION_FAILED: 'OPERATION_FAILED(4)',
    ALREADY_IN_TARGET_STATE: 'ALREADY_IN_TARGET_STATE(101)',
    STATE_TRANSITION_ERROR: 'STATE_TRANSITION_ERROR(102)',
    INCORRECT_TRANSITION: 'INCORRECT_TRANSITION(103)',
}


# spawn 系サービスの追加コード。101/103 などは SetSimulationState の拡張コードと
# 数値が重なるため、result_name とは別の辞書で引く。
SPAWN_RESULT_NAMES = {
    RESULT_FEATURE_UNSUPPORTED: 'FEATURE_UNSUPPORTED(0)',
    RESULT_OK: 'OK(1)',
    RESULT_NOT_FOUND: 'NOT_FOUND(2)',
    RESULT_INCORRECT_STATE: 'INCORRECT_STATE(3)',
    RESULT_OPERATION_FAILED: 'OPERATION_FAILED(4)',
    NAME_NOT_UNIQUE: 'NAME_NOT_UNIQUE(101)',
    102: 'NAME_INVALID(102)',
    UNSUPPORTED_FORMAT: 'UNSUPPORTED_FORMAT(103)',
    NO_RESOURCE: 'NO_RESOURCE(104)',
    105: 'NAMESPACE_INVALID(105)',
    RESOURCE_PARSE_ERROR: 'RESOURCE_PARSE_ERROR(106)',
    MISSING_ASSETS: 'MISSING_ASSETS(107)',
    108: 'UNSUPPORTED_ASSETS(108)',
    109: 'INVALID_POSE(109)',
    ENTITIES_SPAWN_FAILED: 'ENTITIES_SPAWN_FAILED(150)',
}


# world 系サービスの追加コード。101 が SetSimulationState の
# ALREADY_IN_TARGET_STATE と重なるので、こちらも別辞書で引く。
WORLD_RESULT_NAMES = {
    RESULT_FEATURE_UNSUPPORTED: 'FEATURE_UNSUPPORTED(0)',
    RESULT_OK: 'OK(1)',
    RESULT_NOT_FOUND: 'NOT_FOUND(2)',
    RESULT_INCORRECT_STATE: 'INCORRECT_STATE(3)',
    RESULT_OPERATION_FAILED: 'OPERATION_FAILED(4)',
    WORLD_UNSUPPORTED_FORMAT: 'UNSUPPORTED_FORMAT(101) / NO_WORLD_LOADED(101)',
    WORLD_NO_RESOURCE: 'NO_RESOURCE(102)',
    WORLD_RESOURCE_PARSE_ERROR: 'RESOURCE_PARSE_ERROR(103)',
    WORLD_MISSING_ASSETS: 'MISSING_ASSETS(104)',
    WORLD_UNSUPPORTED_ASSETS: 'UNSUPPORTED_ASSETS(105)',
    WORLD_UNSUPPORTED_ELEMENTS: 'UNSUPPORTED_ELEMENTS(106)',
}

# SetEntityState の追加コード
ENTITY_RESULT_NAMES = {
    RESULT_FEATURE_UNSUPPORTED: 'FEATURE_UNSUPPORTED(0)',
    RESULT_OK: 'OK(1)',
    RESULT_NOT_FOUND: 'NOT_FOUND(2)',
    RESULT_INCORRECT_STATE: 'INCORRECT_STATE(3)',
    RESULT_OPERATION_FAILED: 'OPERATION_FAILED(4)',
    INVALID_POSE: 'INVALID_POSE(101)',
}


def result_name(code):
    return RESULT_NAMES.get(code, f'UNKNOWN({code})')


def world_result_name(code):
    return WORLD_RESULT_NAMES.get(code, f'UNKNOWN({code})')


def entity_result_name(code):
    return ENTITY_RESULT_NAMES.get(code, f'UNKNOWN({code})')


def spawn_result_name(code):
    return SPAWN_RESULT_NAMES.get(code, f'UNKNOWN({code})')


def state_name(code):
    return STATE_NAMES.get(code, f'UNKNOWN({code})')


def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# A1 が「無いと話にならない」と見なすサービス。ここに無いものは任意扱いで、
# 実装状況は G1 が get_simulator_features の申告と突き合わせて確かめる。
CORE_SERVICES = (
    'get_simulation_state',
    'set_simulation_state',
    'reset_simulation',
    'spawn_entity',
    'spawn_entities',
    'step_simulation',
    'get_simulator_features',
)

# SimulatorFeatures の値と、それが 1 対 1 で対応するサービス。
# 「申告しているのにサービスが無い」「サービスがあるのに申告していない」の
# 両方を機械的に検出するために使う。フラグだけの機能 (SIMULATION_RESET_TIME や
# ENTITY_BOUNDS_BOX など) は対応するサービスが無いのでここには入れない。
FEATURE_SERVICE_MAP = {
    'SPAWNING': 'spawn_entity',
    'SPAWNING_ENTITIES': 'spawn_entities',
    'DELETING': 'delete_entity',
    'SPAWNABLES': 'get_spawnables',
    'NAMED_POSES': 'get_named_poses',
    'POSE_BOUNDS': 'get_named_pose_bounds',
    'ENTITY_STATE_GETTING': 'get_entity_state',
    'ENTITY_STATE_SETTING': 'set_entity_state',
    'ENTITY_INFO_GETTING': 'get_entity_info',
    'ENTITY_INFO_SETTING': 'set_entity_info',
    'ENTITY_BOUNDS': 'get_entity_bounds',
    'SIMULATION_RESET': 'reset_simulation',
    'SIMULATION_STATE_GETTING': 'get_simulation_state',
    'SIMULATION_STATE_SETTING': 'set_simulation_state',
    'STEP_SIMULATION_SINGLE': 'step_simulation',
    'WORLD_LOADING': 'load_world',
    'WORLD_UNLOADING': 'unload_world',
    'WORLD_INFO_GETTING': 'get_current_world',
    'AVAILABLE_WORLDS': 'get_available_worlds',
}


class ServiceTimeout(Exception):
    """サービス呼び出しが制限時間内に返らなかった。

    リセット後にシミュレータの TCP 接続そのものが壊れると全サービスが
    無応答になるため、この例外は「通信断」の主要な検出手段になる。
    """


class TopicObserver:
    """1 トピックの最新メッセージと受信数を保持する。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.last_msg = None
        self.last_wall_time = None
        self.count = 0

    def on_message(self, msg):
        with self.lock:
            self.last_msg = msg
            self.last_wall_time = time.monotonic()
            self.count += 1

    def snapshot(self):
        with self.lock:
            return self.last_msg, self.last_wall_time, self.count

    def reset_counter(self):
        with self.lock:
            self.count = 0
            return self.last_wall_time


class SimHarness(Node):
    """テストシナリオが使うシミュレータ操作 API。"""

    def __init__(self, profile, service_timeout=15.0):
        super().__init__('simulation_service_test_harness')
        self.profile = profile
        self.service_timeout = service_timeout

        self._srv_clients = {
            'get_simulation_state': self.create_client(
                GetSimulationState, profile.get_state_service),
            'set_simulation_state': self.create_client(
                SetSimulationState, profile.set_state_service),
            'reset_simulation': self.create_client(
                ResetSimulation, profile.reset_service),
            'spawn_entity': self.create_client(
                SpawnEntity, profile.spawn_service),
            'step_simulation': self.create_client(
                StepSimulation, profile.step_service),
            'spawn_entities': self.create_client(
                SpawnEntities, profile.spawn_entities_service),
            'get_simulator_features': self.create_client(
                GetSimulatorFeatures, profile.features_service),
            # --- ここから下は simulation_interfaces の任意サービス -----------
            # 揃っていなくても A1 は落とさない (H 群が個別に確かめる)。
            'delete_entity': self.create_client(
                DeleteEntity, profile.delete_entity_service),
            'get_entities': self.create_client(
                GetEntities, profile.get_entities_service),
            'get_entities_states': self.create_client(
                GetEntitiesStates, profile.get_entities_states_service),
            'get_entity_state': self.create_client(
                GetEntityState, profile.get_entity_state_service),
            'set_entity_state': self.create_client(
                SetEntityState, profile.set_entity_state_service),
            'get_entity_info': self.create_client(
                GetEntityInfo, profile.get_entity_info_service),
            'set_entity_info': self.create_client(
                SetEntityInfo, profile.set_entity_info_service),
            'get_entity_bounds': self.create_client(
                GetEntityBounds, profile.get_entity_bounds_service),
            'get_spawnables': self.create_client(
                GetSpawnables, profile.get_spawnables_service),
            'get_named_poses': self.create_client(
                GetNamedPoses, profile.get_named_poses_service),
            'get_named_pose_bounds': self.create_client(
                GetNamedPoseBounds, profile.get_named_pose_bounds_service),
            'load_world': self.create_client(
                LoadWorld, profile.load_world_service),
            'unload_world': self.create_client(
                UnloadWorld, profile.unload_world_service),
            'get_current_world': self.create_client(
                GetCurrentWorld, profile.get_current_world_service),
            'get_available_worlds': self.create_client(
                GetAvailableWorlds, profile.get_available_worlds_service),
        }

        # ROS-TCP-Endpoint 側の publisher は既定 QoS (RELIABLE / VOLATILE / depth 10)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.joint_states = TopicObserver()
        self.ground_truth = TopicObserver()

        self.create_subscription(
            JointState, profile.joint_states_topic, self.joint_states.on_message, sub_qos)
        self.create_subscription(
            PoseStamped, profile.ground_truth_topic, self.ground_truth.on_message, sub_qos)

        self._command_pub = self.create_publisher(
            JointState, profile.joint_command_topic, 10)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------
    # 後片付け
    # ------------------------------------------------------------------
    def shutdown(self):
        self._executor.shutdown(timeout_sec=2.0)
        try:
            self.destroy_node()
        except Exception:  # noqa: BLE001 - shutdown 時の二重破棄は無視
            pass

    # ------------------------------------------------------------------
    # サービス呼び出し
    # ------------------------------------------------------------------
    def wait_for_services(self, timeout, names=None):
        """サービスが discovery されるまで待ち、見つからなかった名前を返す。

        ``names`` を省略すると CORE_SERVICES だけを見る。任意サービスまで
        必須にすると、それらを実装していないシミュレータで A1 が落ちて
        以降のシナリオが全部 SKIP になってしまうため。
        """
        names = list(CORE_SERVICES if names is None else names)
        deadline = time.monotonic() + timeout
        missing = names
        while missing and time.monotonic() < deadline:
            missing = [
                name for name in missing
                if not self._srv_clients[name].service_is_ready()
            ]
            if missing:
                time.sleep(0.2)
        return missing

    def service_ready(self, name):
        """そのサービスが ROS グラフに現れているか。"""
        client = self._srv_clients.get(name)
        return client is not None and client.service_is_ready()

    def _call(self, name, request, timeout=None):
        timeout = self.service_timeout if timeout is None else timeout
        client = self._srv_clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise ServiceTimeout(f"service '{name}' が {timeout:.0f}s 以内に現れない")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        future.cancel()
        raise ServiceTimeout(f"service '{name}' が {timeout:.0f}s 以内に応答しない")

    def get_state(self, timeout=None):
        return self._call('get_simulation_state', GetSimulationState.Request(), timeout)

    def set_state(self, state, timeout=None):
        req = SetSimulationState.Request()
        req.state.state = state
        return self._call('set_simulation_state', req, timeout)

    def reset(self, scope, timeout=None):
        req = ResetSimulation.Request()
        req.scope = scope
        return self._call('reset_simulation', req, timeout)

    def step(self, steps=1, timeout=None):
        req = StepSimulation.Request()
        # 版によってフィールド名が異なるため、あるものだけ埋める
        for field in ('steps', 'number_of_steps'):
            if hasattr(req, field):
                setattr(req, field, steps)
                break
        return self._call('step_simulation', req, timeout)

    def _fill_pose(self, pose_stamped, pose):
        pose_stamped.header.stamp = TimeMsg()
        pose_stamped.header.frame_id = ''
        pose_stamped.pose.position.x = float(pose[0])
        pose_stamped.pose.position.y = float(pose[1])
        pose_stamped.pose.position.z = float(pose[2])
        qx, qy, qz, qw = euler_to_quaternion(float(pose[3]), float(pose[4]), float(pose[5]))
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

    def _resource(self, urdf_path, with_string=True):
        """simulation_interfaces 2.0.0 以降の Resource を組み立てる。

        uri が空でないとき resource_string は無視される決まりなので、
        両方入れておいても uri が優先される。
        """
        resource = Resource()
        resource.uri = 'file://' + os.path.abspath(urdf_path)
        if with_string and urdf_path.endswith('.urdf'):
            with open(urdf_path, 'r') as f:
                resource.resource_string = f.read()
        return resource

    def spawn(self, urdf_path, name=None, pose=None, namespace='', timeout=None):
        p = self.profile
        req = SpawnEntity.Request()
        req.name = name or p.robot_name
        req.entity_resource = self._resource(urdf_path)
        req.entity_namespace = namespace
        req.allow_renaming = False
        self._fill_pose(req.initial_pose, pose or p.spawn_pose)
        return self._call('spawn_entity', req, timeout)

    # ------------------------------------------------------------------
    # トピック一覧 (名前空間の確認用)
    # ------------------------------------------------------------------
    def list_topics_with_prefix(self, prefix):
        return sorted(name for name, _ in self.get_topic_names_and_types()
                      if name.startswith(prefix))

    def wait_for_topic(self, topic, timeout):
        """トピックが discovery されるまで待つ。

        ROS-TCP-Endpoint 側が publisher を登録して初めて ROS グラフに現れるので、
        スポーン直後は少し待つ必要がある。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(name == topic for name, _ in self.get_topic_names_and_types()):
                return True
            time.sleep(0.3)
        return False

    def spawn_many(self, entries, timeout=None):
        """spawn_entities を叩く。

        entries は ``(name, resource, pose)`` のリスト。resource には URDF の
        パス (str) を渡せば Resource を組み立てるが、結果コードを確かめたい
        ときは Resource をそのまま渡してもよい。
        """
        req = SpawnEntities.Request()
        req.spawn_requests = []
        for name, resource, pose in entries:
            item = SpawnEntityMsg()
            item.name = name
            item.entity_resource = (
                self._resource(resource) if isinstance(resource, str) else resource)
            item.entity_namespace = ''
            item.allow_renaming = False
            self._fill_pose(item.initial_pose, pose)
            req.spawn_requests.append(item)
        return self._call('spawn_entities', req, timeout)

    def spawn_raw(self, resource, name=None, pose=None, timeout=None):
        """不正な Resource を送って結果コードを見るための素の呼び出し。"""
        p = self.profile
        req = SpawnEntity.Request()
        req.name = name if name is not None else p.robot_name
        req.entity_resource = resource
        req.entity_namespace = ''
        req.allow_renaming = False
        self._fill_pose(req.initial_pose, pose or p.spawn_pose)
        return self._call('spawn_entity', req, timeout)

    def simulator_features(self, timeout=None):
        return self._call('get_simulator_features', GetSimulatorFeatures.Request(), timeout)

    # ------------------------------------------------------------------
    # エンティティ系 (simulation_interfaces の任意サービス)
    # ------------------------------------------------------------------
    def delete_entity(self, entity, timeout=None):
        req = DeleteEntity.Request()
        req.entity = entity
        return self._call('delete_entity', req, timeout)

    @staticmethod
    def entity_filters(name_regex='', categories=(), tags=(), tags_mode=0, bounds=None):
        """EntityFilters を組み立てる。省略した項目は「絞り込まない」。"""
        filters = EntityFilters()
        filters.filter = name_regex
        filters.categories = [
            c if isinstance(c, EntityCategory) else EntityCategory(category=int(c))
            for c in categories
        ]
        tags_filter = TagsFilter()
        tags_filter.tags = list(tags)
        tags_filter.filter_mode = tags_mode
        filters.tags = tags_filter
        if bounds is not None:
            filters.bounds = bounds
        return filters

    @staticmethod
    def sphere_bounds(center, radius):
        """Bounds.msg の TYPE_SPHERE。1 点目が中心、2 点目の x が半径。"""
        bounds = Bounds()
        bounds.type = Bounds.TYPE_SPHERE
        first = Vector3Msg()
        first.x, first.y, first.z = (float(v) for v in center)
        second = Vector3Msg()
        second.x = float(radius)
        bounds.points = [first, second]
        return bounds

    def get_entities(self, filters=None, timeout=None):
        req = GetEntities.Request()
        if filters is not None:
            req.filters = filters
        return self._call('get_entities', req, timeout)

    def get_entities_states(self, filters=None, timeout=None):
        req = GetEntitiesStates.Request()
        if filters is not None:
            req.filters = filters
        return self._call('get_entities_states', req, timeout)

    def get_entity_state(self, entity, timeout=None):
        req = GetEntityState.Request()
        req.entity = entity
        return self._call('get_entity_state', req, timeout)

    def set_entity_state(self, entity, pose=None, twist=None, timeout=None,
                         orientation=None):
        """姿勢/速度を書き換える。

        ``pose`` は ``(x, y, z, R, P, Y)``。``orientation`` に
        ``(x, y, z, w)`` を渡すと RPY より優先する (不正な
        クォータニオンを送って INVALID_POSE を確かめるため)。
        """
        req = SetEntityState.Request()
        req.entity = entity
        req.set_pose = pose is not None or orientation is not None
        req.set_twist = twist is not None
        req.set_acceleration = False
        if pose is not None:
            req.state.pose.position.x = float(pose[0])
            req.state.pose.position.y = float(pose[1])
            req.state.pose.position.z = float(pose[2])
            qx, qy, qz, qw = euler_to_quaternion(
                float(pose[3]), float(pose[4]), float(pose[5]))
            req.state.pose.orientation.x = qx
            req.state.pose.orientation.y = qy
            req.state.pose.orientation.z = qz
            req.state.pose.orientation.w = qw
        else:
            req.state.pose.orientation.w = 1.0
        if orientation is not None:
            req.state.pose.orientation.x = float(orientation[0])
            req.state.pose.orientation.y = float(orientation[1])
            req.state.pose.orientation.z = float(orientation[2])
            req.state.pose.orientation.w = float(orientation[3])
        if twist is not None:
            req.state.twist.linear.x = float(twist[0])
            req.state.twist.linear.y = float(twist[1])
            req.state.twist.linear.z = float(twist[2])
            req.state.twist.angular.x = float(twist[3])
            req.state.twist.angular.y = float(twist[4])
            req.state.twist.angular.z = float(twist[5])
        return self._call('set_entity_state', req, timeout)

    def get_entity_info(self, entity, timeout=None):
        req = GetEntityInfo.Request()
        req.entity = entity
        return self._call('get_entity_info', req, timeout)

    def set_entity_info(self, entity, category=None, description='', tags=(), timeout=None):
        req = SetEntityInfo.Request()
        req.entity = entity
        info = EntityInfo()
        info.category = EntityCategory(
            category=int(EntityCategory.CATEGORY_OBJECT if category is None else category))
        info.description = description
        info.tags = list(tags)
        req.info = info
        return self._call('set_entity_info', req, timeout)

    def get_entity_bounds(self, entity, timeout=None):
        req = GetEntityBounds.Request()
        req.entity = entity
        return self._call('get_entity_bounds', req, timeout)

    def get_spawnables(self, sources=(), timeout=None):
        req = GetSpawnables.Request()
        req.sources = list(sources)
        return self._call('get_spawnables', req, timeout)

    def get_named_poses(self, tags=(), tags_mode=0, timeout=None):
        req = GetNamedPoses.Request()
        req.tags.tags = list(tags)
        req.tags.filter_mode = tags_mode
        return self._call('get_named_poses', req, timeout)

    def get_named_pose_bounds(self, name, timeout=None):
        req = GetNamedPoseBounds.Request()
        req.name = name
        return self._call('get_named_pose_bounds', req, timeout)

    # ------------------------------------------------------------------
    # world 系
    # ------------------------------------------------------------------
    def load_world(self, uri='', resource_string='', fail_on_unsupported_element=False,
                   ignore_missing_assets=False, timeout=None):
        req = LoadWorld.Request()
        req.world_resource.uri = uri
        req.world_resource.resource_string = resource_string
        req.fail_on_unsupported_element = fail_on_unsupported_element
        req.ignore_missing_or_unsupported_assets = ignore_missing_assets
        return self._call('load_world', req, timeout)

    def unload_world(self, timeout=None):
        return self._call('unload_world', UnloadWorld.Request(), timeout)

    def get_current_world(self, timeout=None):
        return self._call('get_current_world', GetCurrentWorld.Request(), timeout)

    def get_available_worlds(self, additional_sources=(), tags=(), offline_only=False,
                             continue_on_error=True, timeout=None):
        req = GetAvailableWorlds.Request()
        req.additional_sources = list(additional_sources)
        req.filter.tags = list(tags)
        req.offline_only = offline_only
        req.continue_on_error = continue_on_error
        return self._call('get_available_worlds', req, timeout)

    # ------------------------------------------------------------------
    # 状態遷移のショートカット
    # ------------------------------------------------------------------
    def play(self):
        return self.set_state(SimulationState.STATE_PLAYING)

    def pause(self):
        return self.set_state(SimulationState.STATE_PAUSED)

    def stop(self):
        return self.set_state(SimulationState.STATE_STOPPED)

    def current_state(self):
        return self.get_state().state.state

    # ------------------------------------------------------------------
    # トピック観測
    # ------------------------------------------------------------------
    def wait_for_joint_states(self, timeout, min_count=1):
        """新しい joint_states を ``min_count`` 件受けるまで待つ。

        呼び出し前のカウンタをリセットするので、「デスポーン後に本当に
        publish が止まったか」の判定にもそのまま使える。
        """
        self.joint_states.reset_counter()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, _, count = self.joint_states.snapshot()
            if count >= min_count:
                return True
            time.sleep(0.02)
        return False

    def joint_states_silent_for(self, duration):
        """``duration`` 秒間 joint_states が 1 件も来なければ True。"""
        self.joint_states.reset_counter()
        time.sleep(duration)
        _, _, count = self.joint_states.snapshot()
        return count == 0

    def joint_map(self, timeout=5.0):
        """最新の joint_states を ``{name: (position, velocity)}`` にして返す。"""
        if not self.wait_for_joint_states(timeout):
            return None
        msg, _, _ = self.joint_states.snapshot()
        if msg is None:
            return None
        out = {}
        for i, jname in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else float('nan')
            vel = msg.velocity[i] if i < len(msg.velocity) else float('nan')
            out[jname] = (pos, vel)
        return out

    def sim_time(self, timeout=5.0):
        """joint_states のヘッダから Unity 側 sim 時刻 [s] を取る。"""
        if not self.wait_for_joint_states(timeout):
            return None
        msg, _, _ = self.joint_states.snapshot()
        if msg is None:
            return None
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def base_pose(self, timeout=5.0):
        """最新の ground_truth を ``(x, y, z, yaw)`` にして返す。"""
        self.ground_truth.reset_counter()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, _, count = self.ground_truth.snapshot()
            if count >= 1:
                break
            time.sleep(0.02)
        msg, _, count = self.ground_truth.snapshot()
        if msg is None or count == 0:
            return None
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, p.z, yaw)

    # ------------------------------------------------------------------
    # 指令送信
    # ------------------------------------------------------------------
    def target_joints(self, timeout=5.0):
        """テスト対象にする関節名。プロファイル指定が無ければ実測から取る。"""
        if self.profile.test_joints:
            return list(self.profile.test_joints)
        jm = self.joint_map(timeout)
        if not jm:
            return []
        return sorted(jm.keys())[:self.profile.max_auto_joints]

    def send_command(self, joints, values, duration, rate=20.0):
        """``joints`` へ指令を ``duration`` 秒間送り続ける。

        Unity 側 ``JointStateSub`` は position を xDrive.target に、velocity を
        xDrive.targetVelocity に写すだけなので、保持したい指令は送り続ける
        必要がある (ros2_control 相当の周期送信を自前で行う)。
        """
        msg = JointState()
        msg.name = list(joints)
        if self.profile.command_mode == 'velocity':
            msg.velocity = [float(v) for v in values]
            msg.position = []
        else:
            msg.position = [float(v) for v in values]
            msg.velocity = []
        msg.effort = []

        period = 1.0 / rate
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self._command_pub.publish(msg)
            time.sleep(period)

    def probe_command_response(self, duration=None):
        """指令を送って実際に関節が反応するかを測る。

        戻り値は ``(ok, detail)``。``detail`` には関節ごとの
        「指令前の値 / 指令後の値 / 変化量」を入れて、落ちたときに
        原因が読み取れるようにしてある。
        """
        p = self.profile
        duration = p.command_duration if duration is None else duration

        joints = self.target_joints()
        if not joints:
            return False, 'joint_states が来ないため対象関節を決定できない'

        before = self.joint_map()
        if before is None:
            return False, 'joint_states が来ない (エンティティが存在しない可能性)'

        if p.command_mode == 'velocity':
            values = [p.command_amplitude] * len(joints)
        else:
            # 現在値から一定量ずらした位置を狙う。可動範囲を外れないよう
            # プロファイルの absolute_limit でクランプする。
            values = []
            for j in joints:
                cur = before.get(j, (0.0, 0.0))[0]
                tgt = cur + p.command_amplitude
                if abs(tgt) > p.absolute_limit:
                    tgt = cur - p.command_amplitude
                values.append(max(-p.absolute_limit, min(p.absolute_limit, tgt)))

        self.send_command(joints, values, duration)

        after = self.joint_map()
        if after is None:
            return False, '指令送信後に joint_states が途絶した'

        lines = []
        moved_any = False
        for j, tgt in zip(joints, values):
            b = before.get(j, (float('nan'), float('nan')))
            a = after.get(j, (float('nan'), float('nan')))
            if p.command_mode == 'velocity':
                # 速度指令: 実速度が指令に追従しているか、位置が進んだか
                observed = a[1]
                progressed = abs(a[0] - b[0])
                ok = (abs(observed) > abs(p.command_amplitude) * p.velocity_follow_ratio
                      or progressed > p.motion_threshold)
                lines.append(
                    f'{j}: cmd_vel={tgt:+.3f} obs_vel={observed:+.3f} '
                    f'dpos={progressed:+.3f} -> {"OK" if ok else "NG"}')
            else:
                err = abs(a[0] - tgt)
                moved = abs(a[0] - b[0])
                ok = err < p.position_tolerance or moved > p.motion_threshold
                lines.append(
                    f'{j}: cmd={tgt:+.3f} before={b[0]:+.3f} after={a[0]:+.3f} '
                    f'err={err:.3f} -> {"OK" if ok else "NG"}')
            moved_any = moved_any or ok

        return moved_any, '; '.join(lines)
