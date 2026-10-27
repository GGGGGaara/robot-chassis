#!/usr/bin/env bash
# Snapshot existing evidence only: no CAN socket writes, service starts or stops.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEST="$TASK_DIR/results/support_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$DEST"
{
  date -Is
  id
  cat /proc/sys/kernel/random/boot_id
  cat /proc/1/stat
  uptime
  free -h
  ip -br addr
  ip -details -statistics link show can0 || true
  ps -eo pid,ppid,pgid,lstart,args
  ss -lntup
} > "$DEST/system.txt" 2>&1
dmesg -T > "$DEST/kernel.txt" 2>&1 || true
for name in robot.yaml site_acceptance.json can_feedback_preflight.json runtime_check.json \
    zero_driver_check.json zero_driver_check.log ground_test_forward.json ground_test_left.json \
    ground_test_driver.log driver_readonly_diagnosis.json live_scan_status.json; do
  if [[ -f "$TASK_DIR/$name" ]]; then
    stat "$TASK_DIR/$name" >> "$DEST/file_times.txt"
    cp "$TASK_DIR/$name" "$DEST/"
  fi
done
for name in latest_mission.json offline_completion_20261019.json; do
  [[ ! -f "$TASK_DIR/results/$name" ]] || cp "$TASK_DIR/results/$name" "$DEST/"
done
sha256sum "$TASK_DIR/robot.yaml" "$TASK_DIR/prepare_wsl.sh" \
  "$TASK_DIR/check_can_feedback.py" > "$DEST/sha256.txt"
echo "Support snapshot (may contain older reports; check file_times.txt): $DEST"
