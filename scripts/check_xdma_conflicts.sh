#!/usr/bin/env bash
set -euo pipefail

IFACE_HINT="${1:-}"
FOUND=0

echo "检查占用 /dev/xdma0_h2c_* 或 pcie_uyvy 的进程（不会自动强杀）"

for node in /dev/xdma0_h2c_0 /dev/xdma0_h2c_1 /dev/xdma0_h2c_2; do
  if [[ ! -e "$node" ]]; then
    echo "缺少设备: $node" >&2
    FOUND=1
    continue
  fi
  if command -v fuser >/dev/null 2>&1; then
    if fuser "$node" >/tmp/joint_xdma_fuser.$$ 2>/dev/null; then
      echo "占用 $node:"
      # shellcheck disable=SC2002
      cat /tmp/joint_xdma_fuser.$$ | tr ' ' '\n' | sed '/^$/d' | while read -r pid; do
        [[ -z "$pid" ]] && continue
        ps -p "$pid" -o pid=,user=,cmd= || true
      done
      FOUND=1
    fi
    rm -f /tmp/joint_xdma_fuser.$$
  fi
done

if pgrep -af 'pcie_uyvy|joint_sensor_orchestrator' >/tmp/joint_xdma_pgrep.$$ 2>/dev/null; then
  echo "相关进程:"
  cat /tmp/joint_xdma_pgrep.$$
  if grep -E 'pcie_uyvy' /tmp/joint_xdma_pgrep.$$ >/dev/null; then
    FOUND=1
  fi
fi
rm -f /tmp/joint_xdma_pgrep.$$

if [[ -n "$IFACE_HINT" ]]; then
  echo "提示: 联合雷达网口检查请另跑 ./scripts/prepare_network.sh --check"
fi

if [[ "$FOUND" -ne 0 ]]; then
  echo "发现冲突或缺失，请确认后手动停止占用进程再启动 XDMA。" >&2
  exit 1
fi

echo "未发现 H2C 占用冲突"
exit 0
