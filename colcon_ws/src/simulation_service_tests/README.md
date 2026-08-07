# simulation_service_tests

Unity_ROS2_Robot_Simulator が `simulation_interfaces` のサービスとして公開している機能が
仕様どおりに動くかを、実際に動いているシミュレータへ接続して検証する適合性テストスイート。

とくに **「reset_simulation を呼んだあとロボットが指令を受け付けなくなる」** 種類の
不具合を自動で再現・切り分けできるように作ってある。

## 何を検証するか

| ID | 内容 |
|----|------|
| A1–A3 | 5 サービスの疎通、起動直後の状態、`Result` コードの規約適合 |
| B1–B3 | 状態遷移 (start / 同一状態 / 不正値の拒否) |
| C1–C6 | スポーン、基準状態の記録、ground_truth、**指令が効くことの基準取り**、pause/resume |
| D1–D10 | **reset_simulation の全スコープ**。エンティティ生存、関節・姿勢の復元、リセット後の指令受付、サービス生存、デスポーン、再スポーン、時刻リセット、反復安定性 |
| E1–E2 | `STATE_STOPPED` でのデスポーンと、その後の再スポーン |
| F1–F2 | `step_simulation` の未対応表明、空シーンへのリセット |

★ 印のシナリオ (D5 / D8 / D10) が報告されている不具合の直接検証にあたる。

判定は 5 種類:

- `PASS` — 期待どおり
- `FAIL` — 期待どおりでない (回帰または報告されている不具合)
- `KNOWN_GAP` — 未実装と分かっている項目。既定では終了コードに数えない (`--strict` で数える)
- `SKIP` — 前提シナリオが失敗して実行できなかった
- `ERROR` — シナリオ自体が例外で落ちた

## 使い方

コンテナの **中** で実行する。

### 全部まとめて (推奨)

シミュレータと ROS-TCP-Endpoint の起動・後始末まで面倒を見る:

```bash
cd ~/colcon_ws
colcon build --packages-select simulation_service_tests
source install/setup.bash
./scripts/service_conformance_test.sh                      # 既定は diffbot
./scripts/service_conformance_test.sh --profile servo_demo # 軽い構成
```

終了コード: `0` = すべて期待どおり / `1` = 不具合を検出 / `2` = 実行できなかった。

### 既に起動しているシミュレータに対して

```bash
./scripts/service_conformance_test.sh --no-sim
```

`--no-sim` のときはスタックの起動も後始末もしない。手で立ち上げた
シミュレータへ相乗りしたいとき用。

### 単体で

```bash
ros2 run simulation_service_tests service_conformance --profile servo_demo
ros2 run simulation_service_tests service_conformance --list          # シナリオ一覧
ros2 run simulation_service_tests service_conformance --only D5 D8 D10  # 一部だけ
```

主なオプション:

| オプション | 意味 |
|-----------|------|
| `--profile NAME` | ロボットプロファイル (`diffbot` / `servo_demo` / YAML パス) |
| `--only KEY...` | 指定シナリオだけ実行 |
| `--skip KEY...` | 指定シナリオを除外 |
| `--junit PATH` | JUnit XML を出力 (CI 用) |
| `--json PATH` | JSON レポートを出力 |
| `--strict` | `KNOWN_GAP` も失敗として終了コードに数える |

## プロファイル

`config/*.yaml` にロボットごとの設定と判定しきい値を書く。

- **diffbot** (既定) — 差動二輪。車輪へ速度指令を送るとルートが動くので、
  リセットで「関節が戻るか」と「ルート姿勢が戻るか」を両方検証できる。
- **servo_demo** — 固定台 + 2 関節。センサを積んでいないぶん立ち上がりが速く、
  位置指令の追従がそのまま判定になる。CI 向け。台が固定なので姿勢リセットの
  検証 (C5 / D4) は実質的に無効。

新しいロボットを足すときは `config/` に YAML を 1 枚置くだけでよい。
`xacro_package` / `xacro_file` を書けば実行時に xacro を展開して URDF を作る。

## 設計上の注意

テストを読み書きするときに知っておくべき、シミュレータ側の性質:

- **停止中はトピックが完全に止まる。** `Time.timeScale = 0` の間 Unity の
  `FixedUpdate` は回らないため、`STOPPED` / `PAUSED` では `joint_states` も
  `ground_truth` も publish されない。「エンティティが生きているか」を
  トピックの有無で判定するときは、必ず `PLAYING` にしてから観測すること。
  デスポーン判定 (D7 / E1) が `play()` を挟んでいるのはこのため。
- **`joint_states` のタイムスタンプは sim 時刻。** 一時停止中は進まないので
  タイムアウト計算には使えない。ハーネスは一貫して `time.monotonic()` を使う。
- **指令は送り続ける必要がある。** Unity 側の `JointStateSub` は受信値を
  `xDrive.target` / `targetVelocity` へ写すだけなので、ros2_control 相当の
  周期送信をテスト側で行っている (`send_command`)。
- **ros2_control を経由しない。** `joint_command` トピックへ直接 publish し、
  `joint_states` を直接読む。controller_manager やスポナーを挟まないぶん、
  失敗したときに原因がシミュレータ側だと切り分けやすい。
