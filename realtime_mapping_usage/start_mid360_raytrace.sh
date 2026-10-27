#!/usr/bin/env bash
# Use in Ubuntu-22.04. This starts the real sensor only when this script is run.
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/../tracer_navigation/project_env.sh"
source "$TRACER_MAPPING_VENV/bin/activate"
cd "$TRACER_DIMOS_ROOT"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_TRANSPORT=lcm
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2
unset DIMOS_MID360_PCAP
if pgrep -x fastlio2_native >/dev/null; then
  echo '已有 FAST-LIO2 正在运行。请先在原窗口 Ctrl+C 停止，再启动本脚本。'
  exit 1
fi
LIDAR_ADDRESS="${DIMOS_FASTLIO_LIDAR_IP:-192.168.1.3}"
HOST_ADDRESS="${DIMOS_FASTLIO_HOST_IP:-192.168.1.111}"
echo "雷达: $LIDAR_ADDRESS  主机: $HOST_ADDRESS"
echo '网页查看地址: http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9877%2Fproxy'
echo '先静止初始化约 10 秒，再缓慢移动；Ctrl+C 结束并刷新录制文件。'
exec dimos --transport lcm --rerun-open web \
  --record sqlite --record-topics 'lidar,odometry,tf' \
  run mid360-fastlio-ray-trace \
  --lidar-ip "$LIDAR_ADDRESS" --host-ip "$HOST_ADDRESS" \
  --voxel-size 0.08 --filter-size-surf 0.08 --filter-size-map 0.1 \
  --dense-publish-en false --pointcloud-freq 10 --odom-freq 30 \
  --max-range 30 --emit-every 2 --global-emit-every 10
