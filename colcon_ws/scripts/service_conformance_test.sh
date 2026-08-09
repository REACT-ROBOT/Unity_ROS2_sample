#!/bin/bash
# Unity_ROS2_Robot_Simulator の ROS2 サービス適合性テストを一撃で回すスクリプト。
# コンテナの「中」で実行すること。
#
#   ./scripts/service_conformance_test.sh [オプション]
#
#     --sim-dir DIR     シミュレータ本体の置き場所
#                       (既定: $HOME/Unity_ROS2_Robot_Simulator_v0.9.3_Linux_amd64)
#     --profile NAME    テスト用ロボット (diffbot | servo_demo, 既定: diffbot)
#     --out DIR         レポートとログの出力先 (既定: /tmp/service_conformance)
#     --keep            テスト後もシミュレータを落とさない (手作業で追試したいとき)
#     --strict          KNOWN_GAP も失敗として扱う
#     --no-sim          シミュレータとエンドポイントを起動せず、既に上がっているものを使う
#     ...               残りの引数は service_conformance へそのまま渡す (--only D5 など)
#
# 終了コード: 0 = すべて期待どおり / 1 = 不具合を検出 / 2 = 実行できなかった

set -u

SIM_DIR="${HOME}/Unity_ROS2_Robot_Simulator_v0.9.3_Linux_amd64"
PROFILE="diffbot"
OUT_DIR="/tmp/service_conformance"
KEEP=0
START_SIM=1
EXTRA=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim-dir) SIM_DIR="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --out)     OUT_DIR="$2"; shift 2 ;;
    --keep)    KEEP=1; shift ;;
    --no-sim)  START_SIM=0; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *)         EXTRA+=("$1"); shift ;;
  esac
done

mkdir -p "${OUT_DIR}"
SIM_LOG="${OUT_DIR}/simulator.log"
ENDPOINT_LOG="${OUT_DIR}/endpoint.log"
JUNIT="${OUT_DIR}/${PROFILE}.junit.xml"
JSON="${OUT_DIR}/${PROFILE}.report.json"

log() { echo -e "\033[36m[conformance]\033[0m $*"; }

cleanup_stack() {
  # ros2 launch の子プロセスは launch 本体を殺しても生き残るので個別に落とす
  # パターンは実行ファイルのフルパスで書く。'service_conformance' だけだと
  # このスクリプト自身 (service_conformance_test.sh) にマッチして自殺する。
  pkill -f 'Unity_ROS2_Robot_Simulator.x8[6]_64'                     2>/dev/null
  pkill -f 'default_server_endpoin[t]'                               2>/dev/null
  pkill -f 'simulation_service_tests/service_conformanc[e]'          2>/dev/null
  pkill -f 'ros2_control_nod[e]'                                     2>/dev/null
  pkill -f 'robot_state_publishe[r]'                                 2>/dev/null
  # 強制終了で残った Fast DDS の共有メモリは discovery を壊すため必ず掃除する
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}

# --no-sim のときは他人が上げたスタックへ相乗りするだけなので、
# 起動も後始末もしない (掃除するとそのスタックを巻き添えで殺してしまう)
if [[ ${START_SIM} -eq 1 ]]; then
  trap 'if [[ ${KEEP} -eq 0 ]]; then cleanup_stack; fi' EXIT
  log "前回のスタックを掃除"
  cleanup_stack
  sleep 2
fi

# UDP 限定の DDS プロファイル (共有メモリ経由の discovery 失敗を避ける)
if [[ -f "$(dirname "$0")/fastdds_udp_only.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "$0")" && pwd)/fastdds_udp_only.xml"
fi

# ROS の setup スクリプトは未定義変数を参照するので set -u を一時的に外す
set +u
# ROS_DISTRO はコンテナのベースイメージが設定している。humble / jazzy の
# どちらのコンテナでもそのまま動くよう、決め打ちにしない。
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "${HOME}/colcon_ws/install/setup.sh" ]]; then
  source "${HOME}/colcon_ws/install/setup.sh"
fi
set -u

if ! ros2 pkg prefix simulation_service_tests >/dev/null 2>&1; then
  log "simulation_service_tests がビルドされていない。先に colcon build すること:"
  echo "    cd ~/colcon_ws && colcon build --packages-select simulation_service_tests && source install/setup.bash"
  exit 2
fi

if [[ ${START_SIM} -eq 1 ]]; then
  # エンドポイントを先に上げる。Unity 側は起動時に一度だけ接続し、
  # そのタイミングで publisher/subscriber/service を登録するため、
  # 後から上げると全トピックが行方不明になる。
  log "ROS-TCP-Endpoint を起動"
  # stdin まで切り離す。--keep で残す場合、呼び出し元の端末 (docker exec の
  # パイプ) を掴んだままだと呼び出し側が終了できなくなる。
  nohup ros2 run ros_tcp_endpoint default_server_endpoint \
    --ros-args -p ROS_IP:=0.0.0.0 < /dev/null > "${ENDPOINT_LOG}" 2>&1 &
  sleep 5

  if [[ ! -x "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" ]]; then
    log "シミュレータが見つからない: ${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64"
    exit 2
  fi

  # get_spawnables / get_named_poses / get_available_worlds が返す中身は
  # simulation_resources.json で与える決まりなので、テスト用のものを作って
  # 環境変数で指す。シミュレータは起動時に一度だけ読むため、起動より前に
  # 用意しておく必要がある (--no-sim のときは H7/H8 が空の応答を受け取り、
  # 「設定なし」として扱われる)。
  WORLD_DIR="${OUT_DIR}/worlds"
  # urdf の置き場所は下のテスト実行時に作られるが、GetSpawnables は
  # 呼ばれるたびに走査するので、先に作っておけば「読めないソース」に数えられない。
  mkdir -p "${WORLD_DIR}" "${OUT_DIR}/urdf"
  cat > "${WORLD_DIR}/conformance_world.json" <<'EOF'
{ "objects": [
  { "type": "Cube",   "position": [3.0, 0.5, 0.0], "rotationEuler": [0, 0, 0],
    "scale": [1, 1, 1], "meshPath": "", "isActive": true },
  { "type": "Sphere", "position": [-3.0, 0.5, 0.0], "rotationEuler": [0, 0, 0],
    "scale": [1, 1, 1], "meshPath": "", "isActive": true }
] }
EOF
  RESOURCES_CONFIG="${OUT_DIR}/simulation_resources.json"
  cat > "${RESOURCES_CONFIG}" <<EOF
{
  "spawnable_paths": ["${OUT_DIR}/urdf"],
  "world_paths": ["${WORLD_DIR}"],
  "named_poses": [
    { "name": "conformance_spawn", "description": "適合性テスト用のスポーン地点",
      "tags": ["spawn"], "position": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
      "bounds": { "type": "box", "min": [-0.5, -0.5, 0.0], "max": [0.5, 0.5, 1.0] } },
    { "name": "conformance_goal", "description": "適合性テスト用の目標地点",
      "tags": ["navigation_goal"], "position": [2.0, 1.0, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "bounds": { "type": "sphere", "center": [0.0, 0.0, 0.5], "radius": 1.0 } }
  ]
}
EOF
  export SIMULATION_RESOURCES_CONFIG="${RESOURCES_CONFIG}"
  log "リソース設定を生成: ${RESOURCES_CONFIG}"

  log "シミュレータを起動 (${SIM_DIR})"
  export DISPLAY="${DISPLAY:-:1}"
  export XAUTHORITY="${XAUTHORITY:-/.Xauthority}"
  ( cd "${SIM_DIR}" && nohup ./Unity_ROS2_Robot_Simulator.x86_64 < /dev/null > "${SIM_LOG}" 2>&1 & )
  sleep 14
else
  log "--no-sim: 既に起動しているスタックへ接続する"
fi

log "適合性テストを実行 (profile=${PROFILE})"
ros2 run simulation_service_tests service_conformance \
  --profile "${PROFILE}" \
  --junit "${JUNIT}" \
  --json "${JSON}" \
  --urdf-out-dir "${OUT_DIR}/urdf" \
  "${EXTRA[@]+"${EXTRA[@]}"}" \
  2>&1 | tee "${OUT_DIR}/${PROFILE}.console.log"
STATUS=${PIPESTATUS[0]}

echo
log "レポート: ${JUNIT}"
log "レポート: ${JSON}"
log "シミュレータのログ: ${SIM_LOG}"

# Unity 側の例外はテスト結果からは見えないので、ログから拾って一緒に出す。
# stdout にはブート情報しか出ない。実体は Player.log の方にある。
PLAYER_LOG="${HOME}/.config/unity3d/DefaultCompany/Unity_ROS2_Robot_Simulator/Player.log"
if [[ -f "${PLAYER_LOG}" ]]; then
  cp "${PLAYER_LOG}" "${OUT_DIR}/Player.log" 2>/dev/null
  log "シミュレータの Player.log: ${OUT_DIR}/Player.log"
  EXCEPTIONS=$(grep -c -E 'Exception|registered twice' "${PLAYER_LOG}" 2>/dev/null)
  if [[ "${EXCEPTIONS:-0}" -gt 0 ]]; then
    log "\033[33mPlayer.log に例外/警告が ${EXCEPTIONS} 行ある (種類ごとの先頭):\033[0m"
    grep -E 'Exception|registered twice' "${PLAYER_LOG}" \
      | sed 's/ \[0x[0-9a-f]*\].*//' | sort -u | head -8 | sed 's/^/    /'
  fi
fi

case ${STATUS} in
  0) log "\033[32mすべて期待どおり\033[0m" ;;
  1) log "\033[31m不具合を検出 (上の FAIL / ERROR を参照)\033[0m" ;;
  *) log "\033[31mテストを実行できなかった\033[0m" ;;
esac

if [[ ${KEEP} -eq 1 ]]; then
  log "--keep 指定のためシミュレータは動かしたまま残す"
fi

exit ${STATUS}
