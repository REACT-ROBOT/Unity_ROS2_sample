# Unity_ROS2_sample
[English](README.md) | 日本語

## 概要
このリポジトリは、Unityを使用したROS2連携シミュレータのサンプル実装です。UnityのリアルタイムレンダリングとROS2の通信機能を組み合わせ、ロボット開発やアルゴリズム検証のための評価環境を提供します。

## ブランチ

| ブランチ | 用途 |
|---|---|
| `main` | **ROS 2 Jazzy** を主対象とする開発ライン。既定の distro は jazzy |
| `humble` | ROS 2 Humble 向け。Humble で検証を通した時点のスナップショット |

スクリプトはどちらのブランチでも `${ROS_DISTRO}` を見るので、`main` でも
`./build-dokcer-image.bash humble` と指定すれば Humble で動きます。`humble` ブランチは
Humble 固有の修正が要るときに使います。

> **`humble` ブランチの適合性テストは当時のままです。** `main` で後から足した検証
> (`WORLD_TAGS` 以降の H 群後半、I 群、G6 / F3 / H2b) は入っていません。既存の検証は
> 機能申告で分岐しているので、新しい機能を持つシミュレータに対しても落ちません。
> 揃えたい場合は `main` から cherry-pick してください。この扱いは
> [シミュレータ側の Known-Limitations-ja.md](https://github.com/hijimasa/Unity_ROS2_Robot_Simulator/blob/main/docs/Known-Limitations-ja.md)
> にも保留項目として記載しています。

## 前提条件
- ROS 2 Jazzy (Ubuntu 24.04) または Humble (Ubuntu 22.04)
- Docker (どちらの distro もコンテナ内で完結します)

Unity 自体は不要です。イメージがシミュレータのリリース済み Linux ビルドを
ダウンロードし、コンテナ内で実行します。Unity が要るのはシミュレータを
ソースからビルドするときだけで、それは
[シミュレータ側のリポジトリ](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator)
の話になります。

## インストール方法
1. このリポジトリをサブモジュールごとクローンします：
```
git clone --recursive https://github.com/yourusername/Unity_ROS2_sample.git
```
クローン済みのものは `git submodule update --init --recursive` を実行してください。
シミュレータのサービスや `MagneticGuide` メッセージは `simulation_interfaces` と
`simulation_ros2_utils` から来ており、どちらもサブモジュールで固定してあります。

2. Dockerイメージを作成します。引数で ROS distro を選べます (既定は jazzy)。
```
cd Unity_ROS2_sample/docker
./build-dokcer-image.bash          # ROS 2 Jazzy  / Ubuntu 24.04
./build-dokcer-image.bash humble   # ROS 2 Humble / Ubuntu 22.04
```

3. Dockerコンテナを実行します。ビルド時と同じ distro を指定してください。
```
./run-docker-container.bash
./run-docker-container.bash humble
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
docker exec -it ros-jazzy-unity-sample /bin/bash   # humble なら ros-humble-unity-sample
```
```
./scripts/run_tcp_connector.sh
```

3. 別のターミナルからロボットをスポーンさせます。
```
docker exec -it ros-jazzy-unity-sample /bin/bash   # humble なら ros-humble-unity-sample
```
```
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
```

4. 別のターミナルからteleop_twist_keyboardを実行します。
```
docker exec -it ros-jazzy-unity-sample /bin/bash   # humble なら ros-humble-unity-sample
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

## シミュレータ v1.4.0 で増えた機能を試す

シミュレータのバージョンは `colcon_ws/scripts/simulator_version.txt` で固定していて、
今は **v1.4.0** です。このサンプルでは、その中の 3 つ — LiDAR には映るが衝突しない物体、
AGV 用の磁気ラインセンサとそのテープ、周りの建物なりに測位が劣化する GNSS 受信機 —
を試せるようにしてあります。

diffbot には新しいセンサが既定で載ります。1 フレームあたりの計算を増やしたくない場合は、
xacro に `use_magnetic_guide:=false` / `use_gnss:=false` を渡すと外せます。

[使用方法](#使用方法) の 4 つの端末を立ち上げてから、別の端末で prop を置きます。prop は
`sim_props_description` にあり、どれも URDF エンティティなので `get_entities` に並び、
`delete_entity` で消せ、`reset_simulation` の `SCOPE_SPAWNED` で片付きます。

> `MagneticGuide` メッセージはこのワークスペースでビルドされる
> `simulation_extra_interfaces` のものです。ビルドしたうえで、**シミュレータが接続する前に
> TCP コネクタを再起動**してください。そうしないと型が解決できません。

### LiDAR には映るが、ぶつからない物体

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=weeds
```

スポーン位置の 1.5〜4 m 先に、diffbot の LiDAR のスキャン面より高い草が生えます。
`teleop_twist_keyboard` で突っ込んでみてください。`/diffbot/lidar_link/scan` には実際の
距離で返りが立つのに、ロボットはそのまま通り抜け、`get_contact_events` にも何も残りません。

すり抜けの正体は `<collision_material><sensor_only value="true"/>` で、コリジョン形状が
Unity のトリガになります。レイキャストには当たり、接触解決には入りません。`<collision>` を
持たないリンクとは別物です。LiDAR は物理レイキャストなので、コライダの無いリンクは
単に「無い」のと同じになります。

### 磁気ラインセンサ

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=magnetic_course
ros2 run unity_diffbot_sim magnetic_line_follower
```

コースは 1 周 約21 m の楕円の磁気テープで、脇にマーカが 3 箇所あります。ロボットはその上に
スポーンします。`magnetic_line_follower` は報告された横ずれをそのまま比例制御に入れるだけの
ノード — 実機の AGV でも同じ段です — で、マーカを通過するたびにログに出します。

```
ros2 topic echo /diffbot/magnetic_guide_link/magnetic_guide
```

`position` はテープの横位置 [m] で、ロボットから見て左が正です。`track_positions` には
160 mm のバーの下にあるトラックが全て並ぶので、分岐では 2 本見えます。追従ノードの
`gain` / `linear_speed` / `max_angular` は ROS パラメータです。既定値でコースは回れますが、
詰めた値ではなく出発点として置いてあります。

テープは薄い box に `<collision_material><magnetic_tape polarity="track|marker"/>` を付けた
もので、`magnetic_tape` は `sensor_only` を含意するため踏んで走れます。URDF は
`courses/*.json` と `scripts/gen_tape_urdf.py` から生成しているので、自分のコースは折れ線を
書くだけです。詳しくは
[sim_props_description](colcon_ws/src/sim_props_description/README.md) を参照してください。

### ビル街での GNSS

```
ros2 launch sim_props_description spawn_prop.launch.py prop:=gnss_canyon
```

x=4 から x=26 まで、幅 4 m の街路に沿ってビルが建ちます (途中に 2 m の横道が 2 本)。
そのまま走ると、アンテナは「開空 → 谷間 → 横道 → 谷間」と条件が変わり、等級が順に
落ちていく様子がそのまま出ます。0.3 m/s で直進しながら測ったもの:

| x [m] | 等級 | 使用衛星 | HDOP | σ |
|---|---|---|---|---|
| 2.4 (開空) | RTK Fix | 19 | 0.63 | 2 cm |
| 7.3 | RTK Fix | 5 | 2.0 | 2 cm |
| 9.9 | RTK Float | 8 | 1.0 | 30 cm |
| 14.8 | 単独測位 | 5 | 15.7 | 150 cm |
| 19.8 (横道の脇) | RTK Float | 10 | 0.88 | 30 cm |

街路軸の方向の空は塞げないので、どれだけ深くしても衛星が全て消えることはありません。
劣化を弱めたいときは街路を広げ、強めたいときは狭めます。

```
ros2 topic echo /diffbot/gnss_antenna_link/extended_fix --field status
```

`GPSStatus.status` は RTK Fix (19) と Float (20) を区別できます。`NavSatFix` は構造上これが
できないので、`/diffbot/gnss_antenna_link/fix` では品質が `position_covariance` に載って出ます
(`navsat_transform_node` が読むのはこちらです)。`/diffbot/gnss_antenna_link/nmea` には
GGA/RMC が流れるため、実機で使っている NMEA ドライバをそのまま向けられます。

誤差は真値に振りかけたノイズではありません。遮られた衛星は解から抜け、反射してきた衛星は
回り道の分だけ伸びた行程を持ったまま解に入るので、誤差は**建物で説明できる向きを指し、
同じ場所では同じように出ます**。GUI のエンティティパネルで `<link> gnss rays` を ON にすると
経路が描かれます。緑が直達、太いアンバーが反射です。

## 主な機能
- ROS2トピックによるUnityとの双方向通信
- 物理シミュレーション環境
- センサーデータのシミュレーション
- カスタマイズ可能なロボットモデル

## 謝辞

本プロジェクトでは、MasutaniLab の [choreonoid_ros_khr3](https://github.com/MasutaniLab/choreonoid_ros_khr3) に含まれる KHR3-HV（二足歩行ロボット）のモデルを流用させていただきました。
公開・共有してくださっていることに感謝いたします。

