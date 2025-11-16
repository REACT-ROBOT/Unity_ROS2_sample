# TF Virtual Camera - OBS配信対応版

WSL2環境でもOBS仮想カメラに出力できるようにネットワークストリーム機能を追加しました。

## セットアップ手順

### 1. Windows側（ストリーム受信）を先に起動

#### 必要なソフトウェア
- Python 3.7以上
- OBS Studio with Virtual Camera Plugin

#### Pythonライブラリのインストール

Windows PowerShellまたはコマンドプロンプトで：

```powershell
pip install numpy pyvirtualcam opencv-python
```

#### レシーバースクリプトの実行

```powershell
# WSL2側のファイルをWindowsにコピー
# エクスプローラーで \\wsl$\Ubuntu\home\unity\colcon_ws\install\tf_virtual_camera\share\tf_virtual_camera\scripts\windows_stream_receiver.py をコピー

# または直接実行
python \\wsl$\Ubuntu\home\unity\colcon_ws\install\tf_virtual_camera\share\tf_virtual_camera\scripts\windows_stream_receiver.py --port 5000 --preview
```

**重要**: Windowsファイアウォールで初回実行時にアクセス許可を求められたら「許可」してください。

### 2. WSL2側（ROS 2ノード）を起動

パッケージをビルドして起動：

```bash
cd ~/colcon_ws
colcon build --packages-select tf_virtual_camera
source install/setup.bash

# 自動でWindowsホストIPを検出して送信
ros2 launch tf_virtual_camera tf_camera.launch.py
```

ノードが起動すると、以下のようなログが表示されます：
```
[INFO] [tf_virtual_camera]: Auto-detected Windows host: 10.255.255.254
[INFO] [tf_virtual_camera]: Network stream enabled: 10.255.255.254:5000 (UDP)
```

### 3. OBSで配信

1. OBS Studioを起動
2. **ソース** → **映像キャプチャデバイス** を追加
3. デバイスで **OBS Virtual Camera** を選択
4. VRChatで仮想カメラを選択

## 使用方法

### WSL2でノード起動（デフォルト設定）

```bash
ros2 launch tf_virtual_camera tf_camera.launch.py
```

デフォルトでは：
- 仮想カメラ無効（WSL2では動作しないため）
- ネットワークストリーム有効
- 送信先: 127.0.0.1:5000 (UDP)

### カスタム設定

```bash
ros2 launch tf_virtual_camera tf_camera.launch.py \
    stream_host:=192.168.1.100 \
    stream_port:=6000 \
    update_rate:=60.0
```

### Windows側レシーバー

基本的な起動：
```powershell
python windows_stream_receiver.py
```

オプション付き：
```powershell
python windows_stream_receiver.py --port 5000 --fps 30 --preview
```

パラメータ：
- `--port 5000`: 受信ポート
- `--width 640`: フレーム幅
- `--height 360`: フレーム高さ
- `--fps 30`: 出力FPS
- `--preview`: プレビューウィンドウを表示

## トラブルシューティング

### Windows側で映像が受信できない

#### 1. Windowsファイアウォールの確認

Windows Defenderファイアウォールで、Pythonの受信を許可：

**方法A: 自動（推奨）**
- レシーバー初回起動時にファイアウォールのダイアログが表示されたら「アクセスを許可する」をクリック

**方法B: 手動設定**
1. Windows Defender ファイアウォール → 詳細設定
2. 受信の規則 → 新しい規則
3. ポート → UDP → 5000
4. 接続を許可

#### 2. 送信先IPアドレスの確認

WSL2側でログを確認：
```bash
ros2 launch tf_virtual_camera tf_camera.launch.py
# ログに "Auto-detected Windows host: X.X.X.X" と表示される
```

Windows側でIPアドレスを確認：
```powershell
ipconfig
# "vEthernet (WSL)" のIPv4アドレスを確認
```

両方が一致しない場合、手動指定：
```bash
ros2 launch tf_virtual_camera tf_camera.launch.py stream_host:=<WindowsのIP>
```

#### 3. ポートの競合確認

Windows側でポートが使用されていないか確認：
```powershell
netstat -an | findstr :5000
```

使用中の場合は別のポートを使用：
```bash
# WSL2側
ros2 launch tf_virtual_camera tf_camera.launch.py stream_port:=5001

# Windows側
python windows_stream_receiver.py --port 5001
```

#### 4. UDPパケットのテスト

簡易テスト（WSL2側）：
```bash
echo "test" | nc -u <Windows IP> 5000
```

Windows側でレシーバーが動作していれば受信できるはずです。

## ネットワーク設定

### リモートマシンに送信する場合

```bash
ros2 launch tf_virtual_camera tf_camera.launch.py \
    stream_host:=192.168.1.100 \
    stream_port:=5000
```

受信側（リモートWindows PC）：
```powershell
python windows_stream_receiver.py --port 5000
```

## パフォーマンス

- **解像度**: 640x360 (デフォルト)
- **FPS**: 30 (デフォルト)
- **遅延**: 通常 < 100ms（ローカルネットワーク）
- **帯域**: 約5-10 Mbps（非圧縮RGB）

高FPS（60fps）が必要な場合：
```bash
ros2 launch tf_virtual_camera tf_camera.launch.py update_rate:=60.0
```

Windows側：
```powershell
python windows_stream_receiver.py --fps 60
```

## アーキテクチャ

```
[WSL2 - ROS 2]
  TF Subscriber Node
       ↓
  ShaderMotion Encoder
       ↓
  UDP Stream (chunked)
       ↓
[Windows]
  UDP Receiver
       ↓
  pyvirtualcam → OBS Virtual Camera
       ↓
  OBS Studio → VRChat
```
