#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-real}"
ROBOT_IP="${LUMO_ROBOT_IP:-}"
MODEL="${LUMO_ROBOT_MODEL:-sr_3588}"
DDS_DOMAIN="${LUMO_DDS_DOMAIN:-0}"
DDS_INTERFACE="${LUMO_DDS_INTERFACE:-}"

print_help() {
  cat <<'EOF'
Usage: scripts/run_real.sh [--model sr_3588|sr_thro] [--robot-ip IP] [lumo_real option]

Network options:
  --model MODEL       sr_3588 (default, 192.168.5.2) or sr_thro (172.16.20.2)
  --robot-ip IP       Override the model's default robot address
  --nic INTERFACE     Override wired-interface selection from the robot route
  --domain N          DDS domain, 0..232 (default: 0)
  Environment alternatives: LUMO_ROBOT_MODEL, LUMO_ROBOT_IP,
                            LUMO_DDS_INTERFACE, LUMO_DDS_DOMAIN

lumo_real options:
  --background       Run detached from the launching terminal
  --status           Show background-process status
  --stop             Send SIGTERM to the background process
  --check            Validate assets without connecting to the robot
  --help, -h         Show this help

Command-line network options take precedence over environment variables.
Enter robot debug mode manually before starting. DDS discovers peers on the
selected NIC/domain; --robot-ip does not isolate DDS traffic to one robot.

EOF
}

is_ipv4() {
  local address="$1"
  local octet
  local -a parts

  [[ "${address}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS='.' read -r -a parts <<< "${address}"
  [[ ${#parts[@]} -eq 4 ]] || return 1
  for octet in "${parts[@]}"; do
    [[ "${octet}" =~ ^[0-9]{1,3}$ ]] || return 1
    [[ "${octet}" == "0" || "${octet}" != 0* ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

needs_network=true
manage_process=false
declare -a real_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip|--model|--nic|--domain)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "$1 requires a value." >&2
        exit 2
      fi
      case "$1" in
        --robot-ip) ROBOT_IP="$2" ;;
        --model) MODEL="$2" ;;
        --nic) DDS_INTERFACE="$2" ;;
        --domain) DDS_DOMAIN="$2" ;;
      esac
      shift 2
      ;;
    --robot-ip=*) ROBOT_IP="${1#*=}"; shift ;;
    --model=*) MODEL="${1#*=}"; shift ;;
    --nic=*) DDS_INTERFACE="${1#*=}"; shift ;;
    --domain=*) DDS_DOMAIN="${1#*=}"; shift ;;
    --help|-h)
      print_help
      exit 0
      ;;
    --status|--stop)
      manage_process=true
      needs_network=false
      real_args+=("$1")
      shift
      ;;
    --check)
      needs_network=false
      real_args+=("$1")
      shift
      ;;
    --background)
      real_args+=("$1")
      shift
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 2
      ;;
  esac
done

# Process management must work even if the network environment has changed.
if [[ "${manage_process}" == true ]]; then
  if [[ ! -x "${BUILD_DIR}/lumo_real" ]]; then
    echo "lumo_real was not found. Run scripts/build_real.sh first." >&2
    exit 1
  fi
  exec "${BUILD_DIR}/lumo_real" "${real_args[@]}"
fi

case "${MODEL}" in
  sr_3588) ROBOT_IP="${ROBOT_IP:-192.168.5.2}" ;;
  sr_thro) ROBOT_IP="${ROBOT_IP:-172.16.20.2}" ;;
  *) echo "--model must be sr_3588 or sr_thro." >&2; exit 2 ;;
esac
if [[ ! "${DDS_DOMAIN}" =~ ^[0-9]{1,3}$ ]] || ((10#${DDS_DOMAIN} > 232)); then
  echo "--domain must be an integer in 0..232." >&2
  exit 2
fi
if ! is_ipv4 "${ROBOT_IP}"; then
  echo "Invalid robot IPv4 address: ${ROBOT_IP}" >&2
  exit 2
fi

if [[ ! -x "${BUILD_DIR}/lumo_real" ]]; then
  echo "lumo_real was not found. Run scripts/build_real.sh first." >&2
  exit 1
fi

if [[ "${needs_network}" == true ]]; then
  if [[ -z "${DDS_INTERFACE}" ]]; then
    if ! command -v ip >/dev/null 2>&1; then
      echo "The 'ip' command is required to select the robot network interface." >&2
      exit 1
    fi
    route_line="$(ip -4 route get "${ROBOT_IP}" 2>/dev/null | head -n 1 || true)"
    if [[ "${route_line}" =~ [[:space:]]dev[[:space:]]+([^[:space:]]+) ]]; then
      DDS_INTERFACE="${BASH_REMATCH[1]}"
    else
      echo "No IPv4 route to robot ${ROBOT_IP}. Configure the wired connection first." >&2
      exit 1
    fi
  fi

  if [[ ! "${DDS_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]] ||
     [[ ! -d "/sys/class/net/${DDS_INTERFACE}" ]]; then
    echo "Invalid or unavailable DDS network interface: ${DDS_INTERFACE}" >&2
    exit 1
  fi
  if [[ "${DDS_INTERFACE}" == "lo" || -d "/sys/class/net/${DDS_INTERFACE}/wireless" ]]; then
    echo "Use the wired Ethernet interface connected to the robot." >&2
    exit 1
  fi

  echo "Robot model: ${MODEL} (enter debug mode manually before starting)"
  echo "DDS domain: ${DDS_DOMAIN}"
  echo "Robot IP: ${ROBOT_IP}"
  echo "FastDDS interface: ${DDS_INTERFACE}"
  if command -v ping >/dev/null 2>&1 &&
     ! ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
    echo "Warning: ${ROBOT_IP} did not answer ping; verify the cable and subnet." >&2
  fi
fi

network_args=(--model "${MODEL}" --robot-ip "${ROBOT_IP}" --domain "${DDS_DOMAIN}")
if [[ -n "${DDS_INTERFACE}" ]]; then
  network_args+=(--nic "${DDS_INTERFACE}")
fi
exec "${BUILD_DIR}/lumo_real" "${network_args[@]}" "${real_args[@]}"
