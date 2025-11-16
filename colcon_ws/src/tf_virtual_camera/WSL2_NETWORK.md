# WSL2 ネットワーク設定ガイド

## 問題: WindowsからWSL2のlocalhostにアクセスできない

WSL2では、Docker の `--net=host` を使用しても、Windows側の `127.0.0.1` とWSL2側の `127.0.0.1` は異なるネットワークスタックです。

## 解決策の実装

### 自動検出機能

このパッケージは、WSL2環境でWindowsホストのIPアドレスを自動検出します：

1. `/etc/resolv.conf` の `nameserver` エントリを読み取り
2. これがWindowsホストのIPアドレス（通常は `10.255.255.254` または `172.x.x.1`）
3. UDPパケットをこのIPアドレスに送信

### 起動時のログ例

```
[INFO] [tf_virtual_camera]: Auto-detected Windows host: 10.255.255.254
[INFO] [tf_virtual_camera]: Network stream enabled: 10.255.255.254:5000 (UDP)
```

### 手動でIPを指定する場合

```bash
# Windows側のIPアドレスを確認
cat /etc/resolv.conf | grep nameserver

# 出力例: nameserver 10.255.255.254

# 起動時に明示的に指定
ros2 launch tf_virtual_camera tf_camera.launch.py stream_host:=10.255.255.254
```

## ネットワーク構成

```
┌─────────────────────────────────────┐
│ Windows Host                         │
│  IP: 10.255.255.254 (WSL gateway)   │
│                                      │
│  ┌────────────────────────────┐    │
│  │ Python Receiver            │    │
│  │ Listening: 0.0.0.0:5000    │    │
│  │         ↓                  │    │
│  │ OBS Virtual Camera         │    │
│  └────────────────────────────┘    │
│         ↑ UDP                       │
└─────────┼───────────────────────────┘
          │
┌─────────┼───────────────────────────┐
│ WSL2    │                            │
│  IP: 172.20.138.21                  │
│         │                            │
│  ┌──────┴──────────────────────┐   │
│  │ Docker Container            │   │
│  │ --net=host                  │   │
│  │                             │   │
│  │  ROS 2 Node                 │   │
│  │  Sending to: 10.255.255.254 │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

## よくある問題

### 1. ファイアウォールでブロックされる

**症状**: WSL2側でパケット送信成功、Windows側で受信できない

**解決策**:
```powershell
# Windowsファイアウォールの状態確認
netsh advfirewall show allprofiles

# UDP 5000番ポートを許可（管理者権限）
netsh advfirewall firewall add rule name="ROS2 Stream" dir=in action=allow protocol=UDP localport=5000
```

### 2. IPアドレスが変わる

**症状**: WSL2再起動後に接続できなくなる

**原因**: WSL2のIPアドレスは再起動時に変わることがある

**解決策**: `stream_host:=auto` を使用（デフォルト）。自動で最新のIPを検出します。

### 3. パケットロス

**症状**: 映像がカクカクする、フレームが飛ぶ

**原因**: UDPは信頼性がない

**対策**:
- フレームレートを下げる: `update_rate:=15.0`
- 他のネットワーク負荷を減らす
- WSL2のメモリ制限を調整（`.wslconfig`）

### 4. 遅延が大きい

**原因**: ネットワークスタックのオーバーヘッド

**対策**:
- グリッドサイズを小さくする（データ量削減）
- Windows側で専用の高速ネットワークアダプタを使用
- WSL2のネットワークモードを `mirrored` に変更（Windows 11のみ）

## WSL2のネットワークモード設定（Windows 11）

Windows 11では、WSL2のネットワークモードを変更できます：

`.wslconfig` ファイルを作成（`C:\Users\<YourName>\.wslconfig`）：

```ini
[wsl2]
networkingMode=mirrored
```

これにより、WSL2とWindowsのネットワークスタックが統合され、`localhost` でアクセス可能になります。

**注意**: この機能はWindows 11 22H2以降で利用可能です。

## デバッグコマンド

### WSL2側

```bash
# 送信先IPを確認
cat /etc/resolv.conf | grep nameserver

# WSL2のIPアドレス
hostname -I

# パケット送信テスト
echo "test" > /dev/udp/<Windows IP>/5000

# ノードのログレベルを上げる
ros2 run tf_virtual_camera tf_camera_node --ros-args --log-level debug
```

### Windows側

```powershell
# IPアドレス確認
ipconfig | findstr "WSL"

# ポートリスニング確認
netstat -an | findstr :5000

# UDPパケット受信テスト（PowerShell）
$udpClient = New-Object System.Net.Sockets.UdpClient 5000
$udpClient.Receive([ref]$null)
```
