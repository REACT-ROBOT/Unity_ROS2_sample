# Unity_ROS2_sample
[English](README.md) | 日本語

## 概要
このリポジトリは、Unityを使用したROS2連携シミュレータのサンプル実装です。UnityのリアルタイムレンダリングとROS2の通信機能を組み合わせ、ロボット開発やアルゴリズム検証のための評価環境を提供します。

## 前提条件
- Unity 2022.3 LTS以上
- ROS2 Humble以上
- Ubuntu 22.04 LTS (推奨)

## インストール方法
1. このリポジトリをクローンします：
```
git clone https://github.com/yourusername/Unity_ROS2_sample.git
```

2. Dockerイメージを作成します。
```
cd Unity_ROS2_sample/docker
./build-docker-image.sh
```

3. Dockerコンテナを実行します。
```
./run-docker-container.sh
```

3. ROS2パッケージをビルドします。
```
colcon build
source install/setup.bash
```

## 使用方法
1. Unityでシミュレーションシーンを実行します。
```
./scripts/run_simulator.sh
```

2. 別のターミナルからTCPコネクタを実行します。
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
./scripts/run_tcp_connector.sh
```

3. 別のターミナルからロボットをスポーンさせます。
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. 別のターミナルからteleop_twist_keyboardを実行します。
```
docker exec -it ros-humble-unity-sample /bin/bash
```
```
./scripts/start_sim.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 主な機能
- ROS2トピックによるUnityとの双方向通信
- 物理シミュレーション環境
- センサーデータのシミュレーション
- カスタマイズ可能なロボットモデル

## 謝辞

本プロジェクトでは、MasutaniLab の [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3) に含まれる KHR3-HV（二足歩行ロボット）のモデルを流用させていただきました。
公開・共有してくださっていることに感謝いたします。

