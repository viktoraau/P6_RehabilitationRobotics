#!/usr/bin/env bash
set -euo pipefail

# Clone CANdle repos into a target directory (default: current directory).
# Usage:
#   ./scripts/clone_candle_repos.sh [target_dir]
# Optional environment variables:
#   CANDLE_ROS2_BRANCH=<branch_or_tag>
#   CANDLE_SDK_BRANCH=<branch_or_tag>

TARGET_DIR="${1:-.}"

clone_repo() {
  local name="$1"
  local url="$2"
  local branch="${3:-}"
  local target_path="${TARGET_DIR%/}/${name}"

  if [ -d "${target_path}/.git" ]; then
    echo "[skip] ${name} already exists at ${target_path}"
    return 0
  fi

  echo "[clone] ${name} -> ${target_path}"
  if [ -n "${branch}" ]; then
    git clone --branch "${branch}" --single-branch "${url}" "${target_path}"
  else
    git clone "${url}" "${target_path}"
  fi
}

mkdir -p "${TARGET_DIR}"

clone_repo "candle_ros2" "https://github.com/mabrobotics/candle_ros2.git" "${CANDLE_ROS2_BRANCH:-}"
clone_repo "CANdle-SDK" "https://github.com/mabrobotics/CANdle-SDK.git" "${CANDLE_SDK_BRANCH:-}"

echo "Done. Repositories are ready in: ${TARGET_DIR}"
