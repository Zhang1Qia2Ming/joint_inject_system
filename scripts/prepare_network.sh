#!/usr/bin/env bash
set -euo pipefail

IFACE="${IFACE:-enp3s0f0}"
TARGET_IP="${TARGET_IP:-192.168.1.4}"
STATE_DIR="/run/joint_sensor_player"
STATE_FILE="$STATE_DIR/network.state"
MODE="apply"

usage() {
  cat <<EOF
用法: $0 [--interface IFACE] [--check|--restore]

默认在 enp3s0f0 上添加 192.168.1.31-38/24，不删除已有地址。
--check    只检查链路、地址和域控连通性
--restore  删除本脚本添加的八个雷达地址
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface) IFACE="${2:?--interface缺少参数}"; shift 2 ;;
    --check) MODE="check"; shift ;;
    --restore) MODE="restore"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$MODE" != "check" && "$(id -u)" -ne 0 ]]; then
  if [[ "$MODE" == "restore" ]]; then
    exec sudo "$0" \
      --interface "$IFACE" --restore
  fi
  exec sudo "$0" \
    --interface "$IFACE"
fi

check_network() {
  ip link show "$IFACE" >/dev/null 2>&1 || {
    echo "错误: 网口不存在: $IFACE" >&2
    return 1
  }
  local speed duplex link
  speed="$(ethtool "$IFACE" 2>/dev/null | awk '/Speed:/ {print $2}')"
  duplex="$(ethtool "$IFACE" 2>/dev/null | awk '/Duplex:/ {print $2}')"
  link="$(ethtool "$IFACE" 2>/dev/null | awk '/Link detected:/ {print $3}')"
  echo "$IFACE speed=$speed duplex=$duplex link=$link"
  [[ "$link" == "yes" && "$duplex" == "Full" ]] || return 1
  [[ "${speed%Mb/s}" =~ ^[0-9]+$ && "${speed%Mb/s}" -ge 1000 ]] || return 1

  local missing=0
  for slot in {31..38}; do
    address="192.168.1.$slot"
    if ! ip -4 -o addr show dev "$IFACE" |
        awk '{print $4}' | cut -d/ -f1 | grep -qx "192.168.1.$slot"; then
      echo "缺少源地址 $address"
      missing=1
    elif ! ip route get "$TARGET_IP" from "$address" 2>/dev/null |
        grep -qE "(^| )dev ${IFACE}( |$)"; then
      echo "$address 到 $TARGET_IP 的路由未走 $IFACE"
      missing=1
    fi
  done
  if ping -c 3 -W 1 -I "$IFACE" "$TARGET_IP"; then
    echo "域控 $TARGET_IP 可达"
  else
    echo "域控 $TARGET_IP 不可达" >&2
    return 1
  fi
  return "$missing"
}

case "$MODE" in
  apply)
    install -d -o root -g root -m 0700 "$STATE_DIR"
    [[ ! -L "$STATE_FILE" ]] || {
      echo "错误: 状态文件不能是符号链接" >&2
      exit 1
    }
    ip link set "$IFACE" up
    if [[ ! -e "$STATE_FILE" ]]; then
      (umask 077; printf 'iface=%s\n' "$IFACE" >"$STATE_FILE")
    elif ! grep -qx "iface=$IFACE" "$STATE_FILE"; then
      echo "错误: 状态文件属于其他网口" >&2
      exit 1
    fi
    for slot in {31..38}; do
      address="192.168.1.$slot"
      if ! ip -4 -o addr show dev "$IFACE" |
          awk '{print $4}' | cut -d/ -f1 | grep -qx "$address"; then
        ip addr add "$address/24" dev "$IFACE"
        grep -qx "address=$address" "$STATE_FILE" ||
          echo "address=$address" >>"$STATE_FILE"
      fi
    done
    ip neigh flush dev "$IFACE" >/dev/null 2>&1 || true
    check_network
    ;;
  check)
    check_network
    ;;
  restore)
    if [[ -r "$STATE_FILE" ]]; then
      grep -qx "iface=$IFACE" "$STATE_FILE" || {
        echo "错误: 状态文件网口与 $IFACE 不一致" >&2
        exit 1
      }
      while IFS= read -r line; do
        [[ "$line" == address=* ]] || continue
        address="${line#address=}"
        [[ "$address" =~ ^192\.168\.1\.(3[1-8])$ ]] || {
          echo "错误: 状态文件包含非法地址" >&2
          exit 1
        }
        ip addr del "$address/24" dev "$IFACE" >/dev/null 2>&1 || true
      done <"$STATE_FILE"
      rm -f "$STATE_FILE"
    else
      echo "没有状态文件 $STATE_FILE；未删除任何地址"
    fi
    ;;
esac
