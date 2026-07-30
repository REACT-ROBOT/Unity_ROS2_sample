# サーボモータモデル デモ — 調査・修正レポート (2026-07-30 深夜作業分)

夜間の自律作業で実施した内容のまとめ。結果サマリは末尾。

## 確立した開発ループ

Unity Editor を閉じた状態であれば、ヘッドレス CLI ビルドが可能:

```bash
/home/hijikata/Unity/Hub/Editor/6000.2.7f2/Editor/Unity -batchmode -nographics -quit \
  -projectPath ~/irlab_ws/Unity_ROS2_Robot_Simulator \
  -buildLinux64Player <出力先>/Unity_ROS2_Robot_Simulator.x86_64 -logFile <ログ>
```

約 1〜2 分でフルビルドでき、`docker cp` でコンテナへ配備 → `colcon_ws/scripts/bringup_test.sh`
で起動と健全性チェック（両関節が動いているかまで確認）が回る。

## 今夜見つけて修正したもの

### 1. topic_based_ros2_control の `sum_wrapped_joint_states` (ROS側 / 修正済み)
スポーン過渡のゴミ関節状態をラップ回転と誤検出して恒久オフセット (~2.1 rad) を
ラッチし、以後の指令全体がずれてアームがリミットへ誘導されていた。連続回転しない
このデモでは不要のため、simulation xacro から削除。

### 2. スポーン後の関節状態リセット (Unity側 / SimulationControl.cs / 修正済み)
ランタイム構築中・TeleportRoot のルート回転 (spawn yaw) が関節へステップ入力として
入り、数十〜900 rad/s の速度が注入されて多回転・リミット突破の初期不良を起こしていた。
構築完了とテレポートの後に全 ArticulationBody の jointPosition / jointVelocity /
jointForce と剛体速度をゼロへ明示リセットする処理を追加。

### 3. リミット固着のアンジャム機構 (Unity側 / ServoJointModel.cs / 実装済み)
リミット面上で静止した関節はドライブトルクに応答しなくなる (driveForce は正しい値を
報告するのに動かない)。jointPosition の書き戻しでは解除できず、**ドライブの
lowerLimit/upperLimit を一時的に +10° 広げて拘束を再構築させると解除できる**ことを
実験で確認。境界タッチ検出 → 拡幅 → 範囲内復帰で復元、を実装。

### 4. 仮想ロータのリミットクランプ (Unity側 / ServoJointModel.cs / 実装済み)
仮想ロータ (モデル内部のモータ軸) にはリミットがなく、激しい振りでアームの外側へ
引きずられると伝達バネがアームをリミットへ押し付け続けて戻れなくなる。実サーボの
ホーンがケース端で止まるのと同様に、ロータ角をジョイントリミット範囲へクランプ。

### 5. デモ幾何の変更: q=0 を真下 (吊り下げ平衡) に (ROS側 / URDF / 修正済み)
従来は q=0 が水平 (重力トルク最大)、真下 (±90°) がリミット直上という配置で、
静止点がリミット面に重なりエンジンの固着バグを常時踏んでいた。ジョイント原点を
90° 回して **q=0 = 真下 = 重力平衡** とし、リミット (±1.7 rad) から遠い中央が
静止点になるよう変更。エディタテストが安定していたのも同じ配置だったため。
副次効果: スタンド板との自己衝突圏 (θ≳1.9 rad 相当) にも届かなくなった。

### 6. set_sim_state ユーティリティのバグ (ROS側 / simulation_ros2_utils / 修正済み)
- エラー経路で存在しない `result.message` を参照して AttributeError で落ちる
  → `error_message` に修正
- `ALREADY_IN_TARGET_STATE (101)` をエラー扱いしていたのを情報表示に変更

### 7. bring-up 手順の知見 (scripts/bringup_test.sh に反映)
- **endpoint → シミュレータ → start → launch** の順が安定
  (シミュレータを先に起動すると endpoint への接続・購読登録がレースする)
- 大量にプロセスを kill すると `/dev/shm/fastrtps_*` の残骸で DDS 通信が壊れる
  (`RTPS_TRANSPORT_SHM Error` が指標)。残骸削除 + `ros2 daemon stop/start` で復旧
- JTC スポナーはタイミングで失敗するため launch に respawn=True 設定済み

## 未解決 / 要判断事項

- **エンジンのリミット固着そのもの** はアンジャムで回避しているだけ。根本は
  Unity 6000.2 の ArticulationBody リミット実装の問題の可能性が高く、
  Unity のバージョン更新での再確認か、Unity へのバグ報告を推奨。
- ServoJointModel.cs に一時的なデバッグログ (ServoDbg) が残っている。動作確認後に
  削除して最終ビルドを作ること。
- 変更は全て未コミット。`git diff` で確認の上コミットを。
  - Unity_ROS2_Robot_Simulator: ServoJointModel.cs, SimulationControl.cs
  - Unity_ROS2_sample: servo_demo_description 一式, simulation_ros2_utils,
    scripts/bringup_test.sh ほか

## 今夜さらに見つけたもの (続き)

### 8. アンジャム機構は撤去
リミット拘束の再構築 (limit 書き換え) は関節位置のスナップ (数百〜数千 rad/s) を
誘発し、チャタリングの源になったため撤去。代わりに「静止点をリミットから遠ざける
設計」+「仮想ロータのリミットクランプ」で予防する方針に変更。

### 9. デモ幾何の最終形
- q=0 = 真下 (吊り下げ平衡)。アームリンク自体を -z 向きに定義 (関節フレームの
  回転は使わない: 回転付き関節フレーム + servo_model の組合せで挙動不審があった)
- 振り軸 = x (サーボ突出方向)。振り面がスタンド板と平行になり、板との接触
  (旧配置では |q|~0.19 rad で先端が板に衝突していた) を構造的に回避
- リミット ±1.7 rad、アーム左右間隔 ±0.09 m

### 10. 検証済みパラメータ (servo_model) ※この時点の値。最終調整値は README を参照
```xml
<motor p_gain="9.0" d_gain="0.05" torque_limit="0.18" inertia="2e-3"/>
<friction static="0.005" dynamic="0.003" stribeck_velocity="0.1" viscous="0.08"/>
<backlash width="0.052" stiffness="2.0" damping="0.5"/>
```
伝達剛性は 50 Hz の遅延結合安定条件 (結合振動数 x dt < ~1) から上限が決まる。
ロータ慣性 2e-3 + K=2 でギリギリ安定域。K=400 (物理的実際値) は原理的に不可。

### 11. テスト環境の落とし穴 (--ipc=host)
コンテナは --ipc=host のため、kill された ROS プロセスの FastDDS 共有メモリ残骸が
**ホストの /dev/shm** に蓄積し (100個以上)、DDS ディスカバリを完全に破壊する。
コンテナ再起動でも消えない。対策: bringup_test.sh は UDP-only プロファイル
(scripts/fastdds_udp_only.xml) を使用し、起動毎に残骸を削除。
また `pkill (ros2 launch)` は子ノードを殺さないため、旧スタックの
controller_manager / commander が多重に生き残り指令が混線する。cleanup は
子プロセスまで個別に kill する必要がある (bringup_test.sh に実装済み)。

## 結果サマリ (2026-07-30 早朝)

最終構成での 3 連続クリーンブート + 30 秒動作計測:

- **ideal (青)**: 指令追従誤差 < 0.05 rad で全パターン完璧に追従
- **cheap (赤)**: 全パターンを安定追従しつつ SG90 らしさが出る:
  - スイープ: ideal 比 0.02〜0.05 rad の遅れ/引きずり (バックラッシュ+摩擦)
  - ステップ: 小オーバーシュート (+1.0 指令で 1.02) + バウンスして整定
  - ホールド: ±0.01〜0.03 rad のデッドゾーンオフセット
  - 速度は最大 3.5 rad/s の健全域、発振・固着なし

検証手順: `bash colcon_ws/scripts/bringup_test.sh <シミュレータのディレクトリ>`
(endpoint→sim→start→launch の順で起動し、両関節が実際に動くかまで確認して
CLEAN/BAD を出力)。動作ログ計測は `colcon_ws/scripts/log_servo_demo.py`。

修正済みシミュレータは v0.9.3 として GitHub リリース済み
(docker/dockerfile と scripts/run_simulator.sh も v0.9.3 参照に更新)。
