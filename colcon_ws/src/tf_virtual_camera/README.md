# TF Virtual Camera

TF情報を購読して仮想カメラに出力するROS 2パッケージです。ShaderMotion形式でエンコードし、2台のロボット（各30リンク、合計60リンク）の位置姿勢データを出力します。

## 概要

- `/tf`トピックを購読してロボットのリンク位置姿勢を取得
- CSVファイルで指定されたリンクを追跡（最大30リンク/ロボット）
- 30リンクに満たない場合は(-5, -5, -5)でパディング
- ShaderMotion形式（カラーグリッド）にエンコード
- 仮想カメラ（pyvirtualcam）に出力

## 依存関係

### ROS 2パッケージ
- `rclpy`
- `tf2_ros`
- `geometry_msgs`
- `std_msgs`

### Pythonライブラリ
```bash
pip install pyvirtualcam numpy
```

## ビルド

```bash
cd /home/hijikata/git_ws/Unity_ROS2_sample/colcon_ws
colcon build --packages-select tf_virtual_camera
source install/setup.bash
```

## 使用方法

### 基本的な起動

```bash
ros2 launch tf_virtual_camera tf_camera.launch.py
```

### パラメータ付き起動

```bash
ros2 launch tf_virtual_camera tf_camera.launch.py \
    robot1_name:=KHR3_1 \
    robot2_name:=KHR3_2 \
    link_csv_file:=/path/to/your/links.csv \
    reference_frame:=world \
    update_rate:=30.0
```

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|------------|------------|------|
| `link_csv_file` | `config/khr3_links.csv` | リンク名を定義するCSVファイル |
| `robot1_name` | `KHR3_1` | ロボット1の名前 |
| `robot2_name` | `KHR3_2` | ロボット2の名前 |
| `reference_frame` | `world` | TFルックアップの基準フレーム |
| `update_rate` | `30.0` | 更新レート（Hz） |
| `grid_width` | `80` | カラーグリッドの幅 |
| `grid_height` | `45` | カラーグリッドの高さ |
| `enable_virtual_cam` | `true` | 仮想カメラ出力の有効/無効 |

## CSVファイルフォーマット

CSVファイルは以下の形式で記述します：

```csv
robot1_link1,robot2_link1
robot1_link2,robot2_link2
robot1_link3,robot2_link3
...
```

例：
```csv
KHR3_1_base_footprint,KHR3_2_base_footprint
KHR3_1_base_link,KHR3_2_base_link
KHR3_1_head_link,KHR3_2_head_link
```

- 各行は対応するロボット1とロボット2のリンク名を定義
- 最大30行（30リンク/ロボット）
- 30リンクに満たない場合は自動的に(-5, -5, -5)でパディング

## 出力形式

- **解像度**: 640x360 (grid_width=80, grid_height=45, block_size=8)
- **フレームレート**: 30 FPS（デフォルト）
- **エンコード形式**: ShaderMotion（カラーグリッド）
- **データ構造**: 
  - ヘッダー: 1スロット（シーケンス番号）
  - データ: 60リンク × 2スロット/リンク = 120スロット
  - 合計: 121スロット

## 仮想カメラについて

Linuxでは`v4l2loopback`が必要です：

```bash
# Ubuntu/Debian
sudo apt install v4l2loopback-dkms
sudo modprobe v4l2loopback devices=1

# 確認
ls /dev/video*
```

## トラブルシューティング

### 仮想カメラが起動しない
```bash
# v4l2loopbackを確認
lsmod | grep v4l2loopback

# 手動で読み込み
sudo modprobe v4l2loopback
```

### TFが見つからない
- ロボットが正しくスポーンされているか確認
- `ros2 run tf2_tools view_frames`でTFツリーを確認
- CSVファイルのリンク名がURDFと一致しているか確認

## ライセンス

MIT License
