"""衝突記録 (/get_contact_events) と動く障害物 (motion / SDF actor) の使用例。"""
import json

import pytest

BOX_URDF = """<?xml version="1.0"?>
<robot name="testbox">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.075" ixy="0" ixz="0" iyy="0.075" iyz="0" izz="0.075"/>
    </inertial>
    <visual><geometry><box size="0.3 0.3 0.3"/></geometry></visual>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
</robot>"""

# Unity 座標 (x 右, y 上, z 前) の景観 JSON。ROS の (2, y, z) は Unity の z=2。
# 一辺 1 m の Cube が Unity x -1.5 → 1.5 (= ROS y 1.5 → -1.5) を往復する。
MOVING_WORLD = json.dumps({
    "name": "moving_obstacle_world",
    "objects": [{
        "type": "Cube",
        "position": [-1.5, 0.5, 2.0],
        "rotationEuler": [0, 0, 0],
        "scale": [1, 1, 1],
        "isActive": True,
        "motion": {
            "speed": 1.0,
            "loop": "pingpong",
            "useTimes": False,
            "waypoints": [
                {"position": [-1.5, 0.5, 2.0], "yawDeg": 0, "time": 0},
                {"position": [1.5, 0.5, 2.0], "yawDeg": 0, "time": 0},
            ],
        },
    }],
})


def test_ground_contact_is_recorded_and_filterable(sim):
    """接地は記録されるが、collided() の既定フィルタ (床) では衝突と数えない。"""
    sim.spawn("sitbox", urdf=BOX_URDF, pose=(0.0, 0.0, 0.2))
    sim.play()
    sim.wait(lambda: any(r.other == "Plane" for r in sim.contacts("sitbox")),
             timeout=5.0, message="ground contact recorded")
    assert not sim.collided("sitbox"), "resting on the floor must not count as a collision"


def test_moving_obstacle_hits_entity(sim):
    """動く障害物 (motion 付き景観) が進路上のエンティティへ衝突し、記録される。"""
    sim.load_world_string(MOVING_WORLD)
    try:
        # ROS (2, 0) = Unity (0, y, 2)、Cube の往復経路のちょうど真ん中に置く
        sim.spawn("target", urdf=BOX_URDF, pose=(2.0, 0.0, 0.15))
        sim.play()
        sim.wait(lambda: sim.collided("target"), timeout=10.0,
                 message="moving cube reaches and hits the target")
        others = {r.other for r in sim.contacts("target")}
        assert any("Cube" in o for o in others), f"expected a Cube contact, got {others}"

        # リセットで記録が消え、障害物も始点へ戻る (直後は再衝突していない)
        sim.reset()
        assert sim.contacts("target") == [], "reset must clear contact records"
    finally:
        sim.restore_empty_world()


def test_sdf_actor_world_loads(sim):
    """SDF の <actor> (trajectory サブセット) が動く障害物として読み込める。"""
    sdf = """<?xml version='1.0'?>
    <sdf version='1.7'><world name='actor_world'>
      <actor name='patrol'>
        <link name='body'><visual name='v'>
          <geometry><box><size>1 1 1</size></box></geometry>
        </visual></link>
        <script><loop>true</loop>
          <trajectory id='0' type='walk'>
            <waypoint><time>0</time><pose>2 1.5 0.5 0 0 0</pose></waypoint>
            <waypoint><time>3</time><pose>2 -1.5 0.5 0 0 0</pose></waypoint>
            <waypoint><time>6</time><pose>2 1.5 0.5 0 0 0</pose></waypoint>
          </trajectory>
        </script>
      </actor>
    </world></sdf>"""
    world = sim.load_world_string(sdf)
    assert world.name == "actor_world"
    try:
        sim.spawn("target", urdf=BOX_URDF, pose=(2.0, 0.0, 0.15))
        sim.play()
        sim.wait(lambda: sim.collided("target"), timeout=10.0,
                 message="SDF actor crosses the target and collides")
    finally:
        sim.restore_empty_world()
