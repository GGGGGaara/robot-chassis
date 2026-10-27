#!/usr/bin/env bash
# Explicit operator request only. Delay before the existing guarded single pulse.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# != 2 || "${1:-}" != --execute ]]; then
  echo '用法：ground_test_delayed.sh --execute forward 或 left；未启动测试。'
  exit 2
fi
case "$2" in forward|left) ;; *) echo '方向只能是 forward 或 left。'; exit 2;; esac
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then
  echo '已有底盘驱动，拒绝测试。'; exit 1
fi
# Authenticate before the operator steps away; no embedded credentials.
sudo -v
trap 'echo "倒计时已取消，未启动运动测试。"; exit 130' INT TERM
echo "20 秒后尝试一次 $2 短时测试。请退开停车和转向范围，保持线缆余量，手持物理急停。"
echo 'Ctrl+C 取消。所有原有障碍、反馈及停车检查仍保留。'
for remaining in 20 15 10 5; do
  echo "倒计时：$remaining 秒"
  sleep 5
done
trap - INT TERM
exec bash "$TASK_DIR/ground_test.sh" --execute "$2"
