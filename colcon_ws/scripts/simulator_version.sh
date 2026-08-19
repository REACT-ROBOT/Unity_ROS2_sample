#!/bin/bash
# simulator_version.txt を読んで SIMULATOR_VERSION と SIM_DIR_DEFAULT を定める。
# 各スクリプトから source して使う。バージョンを書くのは .txt だけ。
_sv_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
_sv_file="${_sv_dir}/simulator_version.txt"
if [ ! -f "${_sv_file}" ]; then
  echo "バージョンの定義が見つかりません: ${_sv_file}" >&2
  return 1 2>/dev/null || exit 1
fi
SIMULATOR_VERSION=$(grep -v '^[[:space:]]*#' "${_sv_file}" | grep -v '^[[:space:]]*$' | head -1 | tr -d '[:space:]')
if [ -z "${SIMULATOR_VERSION}" ]; then
  echo "${_sv_file} にバージョンが書かれていません" >&2
  return 1 2>/dev/null || exit 1
fi
SIM_DIR_DEFAULT="${HOME}/Unity_ROS2_Robot_Simulator_${SIMULATOR_VERSION}_Linux_amd64"
