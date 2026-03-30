#!/usr/bin/env bash

# Fast helper utilities for MAB maintenance + controller mode testing.
#
# Usage style 1 (recommended for iterative testing):
#   source mab_ros2_control/scripts/maintenance_fast_test.sh
#   env_setup
#   wait_for_maintenance_services
#   switch_to_position
#   send_position_cmd 0.2 0.0 -0.1
#
# Usage style 2 (single command invocation):
#   ./mab_ros2_control/scripts/maintenance_fast_test.sh quick_joint_check joint_1 1000

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  set -euo pipefail
fi

SERVICE_CALIBRATION="/mab/maintenance/run_calibration"
SERVICE_TORQUE="/mab/maintenance/set_torque_bandwidth"
SERVICE_TESTS="/mab/maintenance/run_drive_tests"

CONTROLLER_MANAGER="/controller_manager"
TRAJ_CONTROLLER="joint_trajectory_controller"
POSITION_CONTROLLER="joint_group_position_controller"
VELOCITY_CONTROLLER="joint_group_velocity_controller"
EFFORT_CONTROLLER="joint_group_effort_controller"

DEFAULT_JOINTS=(joint_1 joint_2 joint_3)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

env_setup() {
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
}

list_maintenance_services() {
  ros2 service list | grep -E "${SERVICE_CALIBRATION}|${SERVICE_TORQUE}|${SERVICE_TESTS}" || true
}

wait_for_service() {
  local service_name="$1"
  local timeout_s="${2:-10}"

  local end_time=$((SECONDS + timeout_s))
  while (( SECONDS < end_time )); do
    if ros2 service type "$service_name" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done

  echo "Timed out waiting for service: $service_name" >&2
  return 1
}

wait_for_maintenance_services() {
  local timeout_s="${1:-15}"
  wait_for_service "$SERVICE_CALIBRATION" "$timeout_s"
  wait_for_service "$SERVICE_TORQUE" "$timeout_s"
  wait_for_service "$SERVICE_TESTS" "$timeout_s"
}

calibrate_joint() {
  local joint_name="$1"
  ros2 service call "$SERVICE_CALIBRATION" mab_ros2_control/srv/RunCalibration "{joint_name: ${joint_name}}"
}

set_torque_bw() {
  local joint_name="$1"
  local bandwidth_hz="$2"
  ros2 service call "$SERVICE_TORQUE" mab_ros2_control/srv/SetTorqueBandwidth "{joint_name: ${joint_name}, bandwidth_hz: ${bandwidth_hz}}"
}

run_drive_tests() {
  local joint_name="$1"
  ros2 service call "$SERVICE_TESTS" mab_ros2_control/srv/RunDriveTests "{joint_name: ${joint_name}}"
}

calibrate_all_joints() {
  local joints=()
  if (( $# > 0 )); then
    joints=("$@")
  else
    joints=("${DEFAULT_JOINTS[@]}")
  fi
  for joint in "${joints[@]}"; do
    echo "=== Calibrating ${joint} ==="
    calibrate_joint "$joint"
  done
}

run_tests_all_joints() {
  local joints=()
  if (( $# > 0 )); then
    joints=("$@")
  else
    joints=("${DEFAULT_JOINTS[@]}")
  fi
  for joint in "${joints[@]}"; do
    echo "=== Running drive tests for ${joint} ==="
    run_drive_tests "$joint"
  done
}

set_torque_all_joints() {
  local bandwidth_hz="$1"
  shift || true

  local joints=()
  if (( $# > 0 )); then
    joints=("$@")
  else
    joints=("${DEFAULT_JOINTS[@]}")
  fi

  for joint in "${joints[@]}"; do
    echo "=== Setting torque bandwidth ${bandwidth_hz} Hz for ${joint} ==="
    set_torque_bw "$joint" "$bandwidth_hz"
  done
}

quick_joint_check() {
  local joint_name="$1"
  local bandwidth_hz="${2:-1000}"

  echo "=== Quick check: ${joint_name} ==="
  set_torque_bw "$joint_name" "$bandwidth_hz"
  run_drive_tests "$joint_name"
  calibrate_joint "$joint_name"
}

quick_all_check() {
  local bandwidth_hz="${1:-1000}"
  shift || true

  local joints=()
  if (( $# > 0 )); then
    joints=("$@")
  else
    joints=("${DEFAULT_JOINTS[@]}")
  fi

  for joint in "${joints[@]}"; do
    quick_joint_check "$joint" "$bandwidth_hz"
  done
}

controller_status() {
  ros2 control list_controllers -c "$CONTROLLER_MANAGER"
}

controller_is_loaded() {
  local controller_name="$1"
  ros2 control list_controllers -c "$CONTROLLER_MANAGER" | grep -qE "^${controller_name}[[:space:]]"
}

ensure_controller_loaded() {
  local controller_name="$1"
  if controller_is_loaded "$controller_name"; then
    return 0
  fi

  ros2 run controller_manager spawner \
    "$controller_name" \
    --controller-manager "$CONTROLLER_MANAGER" \
    --inactive
}

ensure_command_controllers_loaded() {
  ensure_controller_loaded "$TRAJ_CONTROLLER"
  ensure_controller_loaded "$POSITION_CONTROLLER"
  ensure_controller_loaded "$VELOCITY_CONTROLLER"
  ensure_controller_loaded "$EFFORT_CONTROLLER"
}

switch_command_controller() {
  local target="$1"

  local activate=""
  local deactivate=()

  case "$target" in
    trajectory)
      activate="$TRAJ_CONTROLLER"
      deactivate=("$POSITION_CONTROLLER" "$VELOCITY_CONTROLLER" "$EFFORT_CONTROLLER")
      ;;
    position)
      activate="$POSITION_CONTROLLER"
      deactivate=("$TRAJ_CONTROLLER" "$VELOCITY_CONTROLLER" "$EFFORT_CONTROLLER")
      ;;
    velocity)
      activate="$VELOCITY_CONTROLLER"
      deactivate=("$TRAJ_CONTROLLER" "$POSITION_CONTROLLER" "$EFFORT_CONTROLLER")
      ;;
    effort)
      activate="$EFFORT_CONTROLLER"
      deactivate=("$TRAJ_CONTROLLER" "$POSITION_CONTROLLER" "$VELOCITY_CONTROLLER")
      ;;
    *)
      echo "Unknown mode: $target (expected: trajectory|position|velocity)" >&2
      return 1
      ;;
  esac

  ensure_command_controllers_loaded

  ros2 control switch_controllers \
    -c "$CONTROLLER_MANAGER" \
    --deactivate "${deactivate[@]}" \
    --activate "$activate" \
    --best-effort
}

switch_to_trajectory() {
  switch_command_controller trajectory
}

switch_to_position() {
  switch_command_controller position
}

switch_to_velocity() {
  switch_command_controller velocity
}

switch_to_effort() {
  switch_command_controller effort
}

normalize_mode() {
  local mode_in="$1"
  case "$mode_in" in
    traj|trajectory)
      echo "trajectory"
      ;;
    pos|position)
      echo "position"
      ;;
    vel|velocity)
      echo "velocity"
      ;;
    eff|effort)
      echo "effort"
      ;;
    *)
      return 1
      ;;
  esac
}

# Single controller switch entry-point.
# Examples:
#   shift_controller traj
#   shift_controller pos
#   shift_controller vel
shift_controller() {
  local mode
  if ! mode="$(normalize_mode "${1:-}")"; then
    echo "Unknown mode: ${1:-} (expected traj|pos|vel|eff)" >&2
    return 1
  fi
  switch_command_controller "$mode"
}

join_by_comma() {
  local result=""
  local value
  for value in "$@"; do
    if [[ -n "$result" ]]; then
      result+=", "
    fi
    result+="$value"
  done
  printf '%s' "$result"
}

controller_command_topic() {
  local controller_name="$1"

  if ros2 topic list | grep -qx "/${controller_name}/commands"; then
    echo "/${controller_name}/commands"
    return 0
  fi

  if ros2 topic list | grep -qx "/${controller_name}/command"; then
    echo "/${controller_name}/command"
    return 0
  fi

  # default in modern ros2_controllers
  echo "/${controller_name}/commands"
}

send_array_command() {
  local controller_name="$1"
  shift || true

  if (( $# == 0 )); then
    echo "No command values provided." >&2
    return 1
  fi

  local topic
  topic="$(controller_command_topic "$controller_name")"
  local payload
  payload="$(join_by_comma "$@")"

  ros2 topic pub --once "$topic" std_msgs/msg/Float64MultiArray "{data: [${payload}]}"
}

send_position_cmd() {
  send_array_command "$POSITION_CONTROLLER" "$@"
}

send_velocity_cmd() {
  send_array_command "$VELOCITY_CONTROLLER" "$@"
}

send_effort_cmd() {
  send_array_command "$EFFORT_CONTROLLER" "$@"
}

send_trajectory_cmd() {
  if (( $# == 0 )); then
    echo "No trajectory positions provided." >&2
    return 1
  fi

  local expected_count="${#DEFAULT_JOINTS[@]}"
  if (( $# != expected_count )); then
    echo "Trajectory expects ${expected_count} position values (joint_1..joint_3)." >&2
    return 1
  fi

  local positions
  positions="$(join_by_comma "$@")"

  ros2 topic pub --once "/${TRAJ_CONTROLLER}/joint_trajectory" trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [\"joint_1\", \"joint_2\", \"joint_3\"], points: [{positions: [${positions}], time_from_start: {sec: 2, nanosec: 0}}]}"
}

# Single command sender for all command modes.
# Examples:
#   send_cmd pos  0.1 0.0 -0.1
#   send_cmd vel  0.2 0.0  0.0
#   send_cmd traj 0.1 0.0 -0.1
send_cmd() {
  local mode
  if ! mode="$(normalize_mode "${1:-}")"; then
    echo "Unknown mode: ${1:-} (expected traj|pos|vel|eff)" >&2
    return 1
  fi
  shift || true

  case "$mode" in
    trajectory) send_trajectory_cmd "$@" ;;
    position) send_position_cmd "$@" ;;
    velocity) send_velocity_cmd "$@" ;;
    effort) send_effort_cmd "$@" ;;
  esac
}

switch_and_send_position() {
  switch_to_position
  send_position_cmd "$@"
}

switch_and_send_velocity() {
  switch_to_velocity
  send_velocity_cmd "$@"
}

print_usage() {
  cat <<USAGE
Usage:
  $0 env_setup
  $0 list_maintenance_services
  $0 wait_for_maintenance_services [timeout_s]

  $0 calibrate_joint <joint_name>
  $0 set_torque_bw <joint_name> <bandwidth_hz>
  $0 run_drive_tests <joint_name>
  $0 calibrate_all_joints [joint_1 joint_2 ...]
  $0 run_tests_all_joints [joint_1 joint_2 ...]
  $0 set_torque_all_joints <bandwidth_hz> [joint_1 joint_2 ...]
  $0 quick_joint_check <joint_name> [bandwidth_hz]
  $0 quick_all_check [bandwidth_hz] [joint_1 joint_2 ...]

  $0 controller_status
  $0 ensure_command_controllers_loaded
  $0 shift_controller <traj|pos|vel|eff>
  $0 switch_to_trajectory
  $0 switch_to_position
  $0 switch_to_velocity
  $0 switch_to_effort

  $0 send_cmd <traj|pos|vel|eff> <j1> <j2> <j3>
  $0 send_position_cmd <j1> <j2> <j3>
  $0 send_velocity_cmd <j1> <j2> <j3>
  $0 send_effort_cmd <j1> <j2> <j3>
  $0 switch_and_send_position <j1> <j2> <j3>
  $0 switch_and_send_velocity <j1> <j2> <j3>

Tips:
  - Call env_setup first in each shell.
  - Only one command controller should be active at a time.
  - For position/velocity tests, switch mode before sending commands.
USAGE
}

main() {
  local cmd="${1:-}"
  if [[ -z "$cmd" ]]; then
    print_usage
    return 1
  fi
  shift || true

  case "$cmd" in
    env_setup) env_setup "$@" ;;
    list_maintenance_services) list_maintenance_services "$@" ;;
    wait_for_maintenance_services) wait_for_maintenance_services "$@" ;;

    calibrate_joint) calibrate_joint "$@" ;;
    set_torque_bw) set_torque_bw "$@" ;;
    run_drive_tests) run_drive_tests "$@" ;;
    calibrate_all_joints) calibrate_all_joints "$@" ;;
    run_tests_all_joints) run_tests_all_joints "$@" ;;
    set_torque_all_joints) set_torque_all_joints "$@" ;;
    quick_joint_check) quick_joint_check "$@" ;;
    quick_all_check) quick_all_check "$@" ;;

    controller_status) controller_status "$@" ;;
    ensure_command_controllers_loaded) ensure_command_controllers_loaded "$@" ;;
    shift_controller) shift_controller "$@" ;;
    switch_to_trajectory) switch_to_trajectory "$@" ;;
    switch_to_position) switch_to_position "$@" ;;
    switch_to_velocity) switch_to_velocity "$@" ;;
    switch_to_effort) switch_to_effort "$@" ;;

    send_cmd) send_cmd "$@" ;;
    send_position_cmd) send_position_cmd "$@" ;;
    send_velocity_cmd) send_velocity_cmd "$@" ;;
    send_effort_cmd) send_effort_cmd "$@" ;;
    switch_and_send_position) switch_and_send_position "$@" ;;
    switch_and_send_velocity) switch_and_send_velocity "$@" ;;

    *)
      echo "Unknown command: $cmd" >&2
      print_usage
      return 1
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
