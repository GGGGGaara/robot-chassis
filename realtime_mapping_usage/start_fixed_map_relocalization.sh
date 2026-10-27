#!/usr/bin/env bash
# Localization inspection only: no planner, movement module, CAN or cmd_vel bridge.
set -euo pipefail

TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/../tracer_navigation/project_env.sh"
REPO="$TRACER_DIMOS_ROOT"
source "$TRACER_MAPPING_VENV/bin/activate"
cd "$REPO"
export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_TRANSPORT=lcm
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2
unset DIMOS_MID360_PCAP

MAPPING=false
CHECK=false
for option in "$@"; do
  case "$option" in
    --mapping) MAPPING=true ;;
    --check) CHECK=true ;;
    *) echo '用法: start_fixed_map_relocalization.sh [--mapping] [--check]' >&2; exit 2 ;;
  esac
done
GLOBAL_EMIT=0
if $MAPPING; then GLOBAL_EMIT=10; fi

MAP_PATH="${DIMOS_PREMAP:-$TRACER_PREMAP}"
LIDAR_ADDRESS="${DIMOS_FASTLIO_LIDAR_IP:-192.168.1.3}"
HOST_ADDRESS="${DIMOS_FASTLIO_HOST_IP:-192.168.1.111}"

# All options are parsed by the same BlueprintConfigParser used by `dimos run`.
OPTIONS=(
  --g.transport lcm --g.rerun-open web --g.replay false
  --g.simulation '' --g.record ''
  --lidar-ip "$LIDAR_ADDRESS" --host-ip "$HOST_ADDRESS"
  --fastlio2.frame-id odom --sensor-frame-id mid360_link
  --filter-size-surf 0.08 --filter-size-map 0.1
  --dense-publish-en false --pointcloud-freq 10 --odom-freq 30
  --raytracingvoxelmap.world-frame odom
  --voxel-size 0.08 --max-range 30 --emit-every 2 --global-emit-every "$GLOBAL_EMIT"
  --localmaprelocalization.world-frame odom --map-frame map
  --map-file "$MAP_PATH" --relocalize-once true --reloc-interval 3
  --min-local-points 30000
  # The CLI replaces this nested dataclass, so supply its complete preset.
  --relocalize.voxel-coarse 0.59 --relocalize.voxel-fine 0.30
  --relocalize.normal-radius-factor 1.72 --relocalize.fpfh-radius-factor 4.13
  --relocalize.coarse-dist-factor 2.73 --relocalize.ransac-iters 1578291
  --relocalize.mutual-filter true --relocalize.edge-length 0.70
  --relocalize.icp-dist-factor 0.55 --relocalize.icp-stages 2
  --relocalize.orient-normals true
  --relocalize.min-frames 3 --relocalize.max-frames 7
  --relocalize.fitness-threshold 0.8 --relocalize.ransac-restarts 3
  --republish-loaded-map 2 --tf-interval 0.2 --tf-axes 0.2
)

CHECK_OPTIONS=()
if $MAPPING; then CHECK_OPTIONS+=(--mapping-enabled); fi
python "$TASK_DIR/check_fixed_map_relocalization.py" "${CHECK_OPTIONS[@]}" "${OPTIONS[@]}"

if $CHECK; then
  exit 0
fi
if pgrep -x fastlio2_native >/dev/null || pgrep -x voxel_ray_tracing >/dev/null; then
  echo '已有 FAST-LIO2 或光追建图进程，请在它的原终端 Ctrl+C 停止后再运行。' >&2
  exit 1
fi

if $MAPPING; then
  echo '固定参考地图定位＋实时三维建模：全局模型每 10 帧发布，原参考地图只读。'
else
  echo '固定地图重定位测试：原地图只读，光追模块提供临时局部观测，全局地图发布关闭。'
fi
echo '本脚本不包含底盘控制；定位标记代表雷达，尚未加入雷达/底盘安装外参。'
echo "雷达: $LIDAR_ADDRESS  主机: $HOST_ADDRESS"
echo '静止初始化约 10 秒；匹配期间可继续静止等待，首次全局匹配可能较慢。'
echo '先积累至少 30000 个局部点，再尝试匹配；本场景测试接受阈值为 0.8。'
echo '这些参数用于本次排查，不代表所有环境的通用阈值；接受后仍需目视核对。'
echo '若出现 Apply these changes now?，这是库在配置本机 LCM 多播与接收缓存。'
echo '输入 y 并回车；若提示 sudo 密码，请在此终端输入 Ubuntu 密码（输入时不显示字符）。'
echo '等待出现 Rerun 服务启动日志后，再打开下面的网页。'
echo '查看器: http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9877%2Fproxy'
echo '成功日志: align: fitness=... / relocalize lidar: TF ... / placed, no further attempts'
echo '先确认墙角和门口重合，再手动缓慢移动几米。Ctrl+C 结束。'
exec python -m dimos.cli.dimos --transport lcm --rerun-open web \
  run mid360-fastlio-ray-trace local-map-relocalization "${OPTIONS[@]}"
