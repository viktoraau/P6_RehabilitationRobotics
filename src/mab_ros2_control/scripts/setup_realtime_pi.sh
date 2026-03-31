#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-status}"
TARGET_USER="${2:-${SUDO_USER:-${USER}}}"
REALTIME_GROUP="realtime"
LIMITS_FILE="/etc/security/limits.d/90-ros2-control-realtime.conf"

print_status() {
  echo "Realtime status for user: ${TARGET_USER}"
  echo
  echo "Groups:"
  id "${TARGET_USER}" 2>/dev/null || true
  echo
  echo "Current shell limits:"
  echo "  rtprio:  $(ulimit -r)"
  echo "  memlock: $(ulimit -l)"
  echo
  echo "Kernel runtime budget:"
  if [ -r /proc/sys/kernel/sched_rt_runtime_us ]; then
    echo "  sched_rt_runtime_us: $(cat /proc/sys/kernel/sched_rt_runtime_us)"
  else
    echo "  sched_rt_runtime_us: unavailable"
  fi
  echo
  echo "Limits file:"
  if [ -f "${LIMITS_FILE}" ]; then
    cat "${LIMITS_FILE}"
  else
    echo "  missing: ${LIMITS_FILE}"
  fi
}

apply_config() {
  if [ "${EUID}" -ne 0 ]; then
    echo "Run with sudo to apply realtime limits:"
    echo "  sudo $0 apply ${TARGET_USER}"
    exit 1
  fi

  groupadd -f "${REALTIME_GROUP}"
  usermod -aG "${REALTIME_GROUP}" "${TARGET_USER}"

  cat > "${LIMITS_FILE}" <<'EOF'
@realtime   -  rtprio   99
@realtime   -  nice    -20
@realtime   -  memlock unlimited
EOF

  echo "Realtime limits written to ${LIMITS_FILE}"
  echo "User '${TARGET_USER}' added to group '${REALTIME_GROUP}'"
  echo "Log out and back in before retesting ros2_control."
}

case "${MODE}" in
  status)
    print_status
    ;;
  apply)
    apply_config
    ;;
  *)
    echo "Usage: $0 [status|apply] [user]"
    exit 1
    ;;
esac
