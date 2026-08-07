# Unity_ROS2_sample
[English](README.md) | 日本語

## 概要
このリポジトリは、Unityを使用したROS2連携シミュレータのサンプル実装です。UnityのリアルタイムレンダリングとROS2の通信機能を組み合わせ、ロボット開発やアルゴリズム検証のための評価環境を提供します。

## 前提条件
- Unity 2022.3 LTS以上
- ROS 2 Humble (Ubuntu 22.04) または Jazzy (Ubuntu 24.04)
- Docker (どちらの distro もコンテナ内で完結します)

## インストール方法
1. このリポジトリをクローンします：
```
git clone https://github.com/yourusername/Unity_ROS2_sample.git
```

2. Dockerイメージを作成します。引数で ROS distro を選べます (既定は humble)。
```
cd Unity_ROS2_sample/docker
./build-dokcer-image.bash          # ROS 2 Humble / Ubuntu 22.04
./build-dokcer-image.bash jazzy    # ROS 2 Jazzy  / Ubuntu 24.04
```

3. Dockerコンテナを実行します。ビルド時と同じ distro を指定してください。
```
./run-docker-container.bash
./run-docker-container.bash jazzy
```

コンテナ名は `ros-<distro>-unity-sample` なので、humble と jazzy を並行して置けます。

3. ROS2パッケージをビルドします。
```
colcon build
source install/setup.bash
```

> **注意**: `colcon_ws` は distro 間で共有できません。humble と jazzy を切り替えるときは
> 先に成果物を消してください。Python のバージョン (3.10 / 3.12) が違うため、
> 残っているとメッセージ型の読み込みで
> `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c'` になります。
> ```
> rm -rf build install log && colcon build
> ```

## 使用方法
1. Unityでシミュレーションシーンを実行します。
```
./scripts/run_simulator.sh
```

2. 別のターミナルからTCPコネクタを実行します。
```
docker exec -it ros-humble-unity-sample /bin/bash   # jazzy なら ros-jazzy-unity-sample
```
```
./scripts/run_tcp_connector.sh
```

3. 別のターミナルからロボットをスポーンさせます。
```
docker exec -it ros-humble-unity-sample /bin/bash   # jazzy なら ros-jazzy-unity-sample
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. 別のターミナルからteleop_twist_keyboardを実行します。
```
docker exec -it ros-humble-unity-sample /bin/bash   # jazzy なら ros-jazzy-unity-sample
```
```
./scripts/start_sim.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## サービス適合性テスト
シミュレータが `simulation_interfaces` のサービスとして公開している機能
(`spawn_entity` / `set_simulation_state` / `get_simulation_state` / `reset_simulation` /
`step_simulation`) が仕様どおり動くかを自動検証できます。とくに
「`reset_simulation` を呼んだあとロボットが指令を受け付けなくなる」種類の不具合を
再現・切り分けするために用意しています。

コンテナ内で以下を実行します。
```
cd ~/colcon_ws
colcon build --packages-select simulation_service_tests simulation_ros2_utils
source install/setup.bash
./scripts/service_conformance_test.sh
```

ROS-TCP-Endpoint とシミュレータの起動から後始末までスクリプトが行います。
終了コードは 0 = すべて期待どおり / 1 = 不具合を検出 / 2 = 実行できなかった、です。

詳細は [colcon_ws/src/simulation_service_tests/README.md](colcon_ws/src/simulation_service_tests/README.md) を参照してください。

## 主な機能
- ROS2トピックによるUnityとの双方向通信
- 物理シミュレーション環境
- センサーデータのシミュレーション
- カスタマイズ可能なロボットモデル

## 謝辞

本プロジェクトでは、MasutaniLab の [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3) に含まれる KHR3-HV（二足歩行ロボット）のモデルを流用させていただきました。
公開・共有してくださっていることに感謝いたします。

