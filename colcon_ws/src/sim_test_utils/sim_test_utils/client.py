"""Unity_ROS2_Robot_Simulator に対するテストシナリオ用クライアント。

simulation_interfaces のサービス群を pytest から使いやすい形に包む。
決定論が要る検証は PAUSED + step() (step_simulation) で書くこと。
"""
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import (
    DeleteEntity, GetEntities, GetEntityState, SetEntityState,
    LoadWorld, UnloadWorld, ResetSimulation, SetSimulationState,
    SpawnEntity, StepSimulation)
from simulation_extra_interfaces.srv import ApplyLinkWrench, GetContactEvents


class ServiceError(RuntimeError):
    """サービスが RESULT_OK 以外を返した。"""


class SimClient:
    """シミュレータ 1 台ぶんのサービスクライアント。

    spawn() したエンティティ名を控えるので、テスト後の後始末
    (cleanup_spawned) をフィクスチャに任せられる。
    """

    def __init__(self, node: Node = None, service_timeout: float = 20.0):
        self._own_node = node is None
        self.node = node or Node("sim_test_client")
        self.service_timeout = service_timeout
        self.spawned = []
        self._clients = {}

    # ------------------------------------------------------------------
    # 低レベル
    # ------------------------------------------------------------------

    def _client(self, srv_type, name):
        key = (srv_type, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(srv_type, name)
        return self._clients[key]

    def call(self, srv_type, name, request, timeout: float = None):
        """サービスを呼び、応答をそのまま返す (結果コードは見ない)。"""
        timeout = timeout or self.service_timeout
        client = self._client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise ServiceError(f"service {name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if future.result() is None:
            raise ServiceError(f"service {name} timed out")
        return future.result()

    @staticmethod
    def _check(name, result):
        # RESULT_OK = 1 (0 は旧実装の互換)
        if result.result not in (0, 1):
            raise ServiceError(
                f"{name} failed: result={result.result} '{result.error_message}'")

    def close(self):
        if self._own_node:
            self.node.destroy_node()

    # ------------------------------------------------------------------
    # エンティティ
    # ------------------------------------------------------------------

    def spawn(self, name, urdf: str = None, urdf_path: str = None,
              pose=(0.0, 0.0, 0.0), yaw: float = 0.0, namespace: str = ""):
        """URDF 文字列またはファイルパスからエンティティをスポーンする。"""
        req = SpawnEntity.Request()
        req.name = name
        req.entity_namespace = namespace
        if urdf is not None:
            req.entity_resource.resource_string = urdf
        elif urdf_path is not None:
            req.entity_resource.uri = urdf_path if "://" in urdf_path \
                else f"file://{urdf_path}"
        else:
            raise ValueError("either urdf or urdf_path is required")
        p = req.initial_pose.pose
        p.position.x, p.position.y, p.position.z = map(float, pose)
        p.orientation.z = math.sin(yaw / 2.0)
        p.orientation.w = math.cos(yaw / 2.0)
        res = self.call(SpawnEntity, "/spawn_entity", req, timeout=60.0)
        self._check("spawn_entity", res.result)
        self.spawned.append(name)
        return name

    def delete(self, name):
        req = DeleteEntity.Request()
        req.entity = name
        res = self.call(DeleteEntity, "/delete_entity", req)
        self._check("delete_entity", res.result)
        if name in self.spawned:
            self.spawned.remove(name)

    def entities(self):
        res = self.call(GetEntities, "/get_entities", GetEntities.Request())
        return list(res.entities)

    def state(self, name):
        req = GetEntityState.Request()
        req.entity = name
        res = self.call(GetEntityState, "/get_entity_state", req)
        self._check("get_entity_state", res.result)
        return res.state

    def position(self, name):
        """ROS 座標系の (x, y, z)。"""
        p = self.state(name).pose.position
        return (p.x, p.y, p.z)

    def teleport(self, name, pose=(0.0, 0.0, 0.0), yaw: float = 0.0):
        req = SetEntityState.Request()
        req.entity = name
        p = req.state.pose
        p.position.x, p.position.y, p.position.z = map(float, pose)
        p.orientation.z = math.sin(yaw / 2.0)
        p.orientation.w = math.cos(yaw / 2.0)
        res = self.call(SetEntityState, "/set_entity_state", req)
        self._check("set_entity_state", res.result)

    def cleanup_spawned(self):
        """spawn() で作ったエンティティを全部消す (フィクスチャ用)。"""
        for name in list(self.spawned):
            try:
                self.delete(name)
            except ServiceError:
                self.spawned.remove(name)

    # ------------------------------------------------------------------
    # シミュレーションの状態と時間
    # ------------------------------------------------------------------

    def _set_state(self, state):
        req = SetSimulationState.Request()
        req.state = SimulationState(state=state)
        res = self.call(SetSimulationState, "/set_simulation_state", req)
        self._check("set_simulation_state", res.result)

    def play(self):
        self._set_state(SimulationState.STATE_PLAYING)

    def pause(self):
        self._set_state(SimulationState.STATE_PAUSED)

    def stop(self):
        self._set_state(SimulationState.STATE_STOPPED)

    def step(self, steps: int):
        """PAUSED 状態から steps 物理ステップだけ進める (完了までブロック)。"""
        req = StepSimulation.Request()
        req.steps = int(steps)
        res = self.call(StepSimulation, "/step_simulation", req, timeout=120.0)
        self._check("step_simulation", res.result)

    def step_seconds(self, seconds: float, physics_hz: float = 50.0):
        """物理レートを引数で明示してシミュレーション秒数ぶん進める。"""
        self.step(max(1, round(seconds * physics_hz)))

    def reset(self, scope=ResetSimulation.Request.SCOPE_STATE):
        req = ResetSimulation.Request()
        req.scope = scope
        res = self.call(ResetSimulation, "/reset_simulation", req)
        self._check("reset_simulation", res.result)

    # ------------------------------------------------------------------
    # ワールド
    # ------------------------------------------------------------------

    def load_world(self, path_or_uri: str):
        req = LoadWorld.Request()
        req.world_resource.uri = path_or_uri if "://" in path_or_uri \
            else f"file://{path_or_uri}"
        res = self.call(LoadWorld, "/load_world", req, timeout=60.0)
        self._check("load_world", res.result)
        return res.world

    def load_world_string(self, content: str):
        """シーン JSON または SDF (< で始まる) の文字列からワールドを載せ替える。"""
        req = LoadWorld.Request()
        req.world_resource.resource_string = content
        res = self.call(LoadWorld, "/load_world", req, timeout=60.0)
        self._check("load_world", res.result)
        return res.world

    def restore_empty_world(self):
        """景観を空に戻す (組み込みの床とライトは常に残る)。"""
        self.load_world_string('{"name":"empty","objects":[]}')

    def unload_world(self):
        res = self.call(UnloadWorld, "/unload_world", UnloadWorld.Request())
        self._check("unload_world", res.result)

    # ------------------------------------------------------------------
    # 外乱注入
    # ------------------------------------------------------------------

    def apply_wrench(self, entity, link: str = "",
                     force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0),
                     duration: float = 0.0):
        """リンク重心へワールド座標系 (ROS 慣習) のレンチを duration シミュ秒印加する。"""
        req = ApplyLinkWrench.Request()
        req.entity = entity
        req.link = link
        w = Wrench()
        w.force.x, w.force.y, w.force.z = map(float, force)
        w.torque.x, w.torque.y, w.torque.z = map(float, torque)
        req.wrench = w
        req.duration = float(duration)
        res = self.call(ApplyLinkWrench, "/apply_link_wrench", req)
        if res.result != ApplyLinkWrench.Response.RESULT_OK:
            raise ServiceError(
                f"apply_link_wrench failed: result={res.result} '{res.error_message}'")

    # ------------------------------------------------------------------
    # 衝突記録
    # ------------------------------------------------------------------

    def contacts(self, entity: str = "", clear: bool = False):
        """(entity, link, other) ごとの衝突記録リストを返す。"""
        req = GetContactEvents.Request()
        req.entity = entity
        req.clear = clear
        res = self.call(GetContactEvents, "/get_contact_events", req)
        if res.result != GetContactEvents.Response.RESULT_OK:
            raise ServiceError(
                f"get_contact_events failed: result={res.result} '{res.error_message}'")
        return list(res.contacts)

    def collided(self, entity: str, ignore=("Plane",)):
        """entity が ignore 以外の何かとぶつかった記録があるか。

        ignore の既定 "Plane" は組み込みの床。車輪やベースの接地は正常なので
        除外し、それ以外との接触を「衝突」と数える。
        """
        return any(
            not any(pattern in record.other for pattern in ignore)
            for record in self.contacts(entity))

    # ------------------------------------------------------------------
    # 待ち合わせ
    # ------------------------------------------------------------------

    def wait(self, condition, timeout: float = 10.0, period: float = 0.1,
             message: str = "condition"):
        """condition() が真になるまで待つ (実時間タイムアウト)。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=period)
            if condition():
                return True
        raise TimeoutError(f"timed out waiting for {message}")
