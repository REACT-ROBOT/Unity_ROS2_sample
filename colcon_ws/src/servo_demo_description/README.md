# servo_demo_description

Unity_ROS2_Robot_Simulator v0.9.3 のサーボモータモデル
(Stribeck 摩擦 + バックラッシ) のデモ。

固定スタンドに 2 個のサーボ + 振り子アームを並べ、同じ指令を与えて比較する。
アームは q=0 で真下 (重力平衡) を向き、振り子のように左右へ振れる。

- **青 (ideal_joint)** : 理想的な位置サーボ (素の xDrive、臨界減衰)
- **赤 (cheap_joint)** : SG90 のような安物サーボ (`<servo_model>` 適用)
  - ステップ指令でオーバーシュート + リンギング
  - 低速移動でスティックスリップ (固着 → breakaway ジャンプ)
  - バックラッシ (全幅 0.09 rad) による荷重側フランクへの張り付きと
    真下通過時のギャップ横断、指令反転時のガタ

## 使い方

シミュレータと ros_tcp_endpoint を起動した後:

```bash
ros2 launch servo_demo_description servo_demo_spawn.launch.py
```

`servo_demo_commander.py` が「ステップ → ステップ → 低速スイープ → 低速スイープ → 原点」
を繰り返し送信する。起動からヘルスチェックまで自動化したスクリプトもある:

```bash
bash colcon_ws/scripts/bringup_test.sh   # endpoint→sim→start→launch の順に起動し CLEAN/BAD を判定
```

## モデルが再現する現象 (実測)

![validation](doc/servo_model_validation.png)

- **左: バックラッシ**。cheap−ideal の差は重力が押し付けるギヤフランク側に
  ±45 mrad (= 設定した全幅 0.09 rad の半分) で張り付き、真下 (q=0) で
  90 mrad のギャップを横断して反対フランクへ乗り移る。上り/下りの枝の分離が
  ヒステリシスループ。
- **右: スティックスリップ**。低速スイープ (0.11 rad/s) 中に約 0.7 s の固着と
  breakaway ジャンプを繰り返す。ジャダー振幅はおおよそ
  (静止摩擦 − 動摩擦) / 直列剛性 で設計できる。

全シーケンスの追従比較は [doc/servo_demo_comparison.png](doc/servo_demo_comparison.png)、
計測データは [doc/data/](doc/data/) (log_servo_demo.py で記録)、
図の再生成は `colcon_ws/scripts/plot_servo_validation.py`。

## servo_model の記述 (robot 直下)

```xml
<servo_model joint="cheap_joint">
  <motor p_gain="4.0" d_gain="0.02" torque_limit="0.18" inertia="2e-3"/>
  <friction static="0.06" dynamic="0.015" stribeck_velocity="0.1" viscous="0.08"/>
  <backlash width="0.09" stiffness="2.0" damping="0.5"/>
</servo_model>
```

| 要素/属性 | 意味 | 単位 |
|---|---|---|
| motor/p_gain, d_gain | サーボ内部 PD ゲイン (省略時は joint の `<drive>` 値) | N·m/rad, N·m/(rad/s) |
| motor/torque_limit | モータトルク上限 (SG90 のストール 1.8 kgf·cm ≈ 0.18) | N·m |
| motor/inertia | ギヤ比換算のロータ慣性 | kg·m² |
| friction/static, dynamic | 静止摩擦 (breakaway) / クーロン摩擦 | N·m |
| friction/stribeck_velocity | Stribeck ピークの減衰速度 | rad/s |
| friction/viscous | 粘性摩擦。無負荷速度 ≈ torque_limit / viscous の近似にもなる | N·m/(rad/s) |
| backlash/width | バックラッシ全幅 (デッドバンド) | rad |
| backlash/stiffness, damping | 伝達系 (ギヤ) 剛性・減衰 | N·m/rad, N·m/(rad/s) |

## チューニングの目安

- **オーバーシュート/リンギング**: `d_gain` を下げると増える。到達時の運動量は
  粘性で頭打ちされた速度で決まるため、`viscous` を下げると突入が激しくなる
- **スティックスリップ**: 振幅 ≈ (static − dynamic) / K_series、
  K_series = 1/(1/p_gain + 1/stiffness)。`static` を上げるか `p_gain` を
  下げると顕著になる
- **バックラッシ**: `width` がそのままフランク間ギャップになる。強い摩擦の
  ジャダーはギャップを均してヒステリシスを痩せさせる (実機同様)

## 制約・注意 (シミュレータ実装由来)

- 伝達剛性 `backlash/stiffness` は 50 Hz 物理更新の遅延結合安定条件で上限が
  決まる (結合振動数 × Δt < ~1)。この構成では K=2 程度が実用上限で、
  実機相当の数百 N·m/rad は発振する
- URDF の `<limit effort>` と `<drive stiffness/damping>` はエンジン側で
  π/180 倍にスケールされるため、SI 値に ×180/π を掛けて記述している
  (URDF 内コメント参照)
- 重力静止姿勢はジョイントリミットから離すこと (リミット面で静止した関節が
  ドライブに応答しなくなるエンジン挙動がある)。詳細は
  [doc/DEVELOPMENT_NOTES.md](doc/DEVELOPMENT_NOTES.md)
