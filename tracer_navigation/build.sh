#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
SOURCE="$TRACER_VENDOR_ROOT"
RUNTIME="$TRACER_RUNTIME_ROOT"
[[ -f "$SOURCE/sdk/CMakeLists.txt" && -d "$SOURCE/ros2/tracer_msgs" ]] || { echo 'Missing vendor SDK/messages; see docs/team/QUICKSTART.md'; exit 1; }
# rosidl's Humble generation tools do not handle the Chinese workspace path.
# Build managed copies under an ASCII Linux path; never modify the supplied SDK.
mkdir -p "$RUNTIME/vendor/sdk" "$RUNTIME/ws/src/tracer_msgs" "$RUNTIME/ws/src/tracer_navigation_driver"
cp -a "$SOURCE/sdk/." "$RUNTIME/vendor/sdk/"
cp -a "$SOURCE/ros2/tracer_msgs/." "$RUNTIME/ws/src/tracer_msgs/"
cp -a "$TASK_DIR/ws/src/tracer_navigation_driver/." "$RUNTIME/ws/src/tracer_navigation_driver/"
mkdir -p "$RUNTIME/ws/src/tracer_bringup"
cp -a "$TASK_DIR/ws/src/tracer_bringup/." "$RUNTIME/ws/src/tracer_bringup/"
cmake -S "$RUNTIME/vendor/sdk" -B "$RUNTIME/sdk_build" \
  -DCMAKE_INSTALL_PREFIX="$RUNTIME/sdk_install" -DBUILD_TESTING=OFF
cmake --build "$RUNTIME/sdk_build" -j4
cmake --install "$RUNTIME/sdk_build"
cd "$RUNTIME/ws"
colcon build --packages-select tracer_msgs tracer_navigation_driver tracer_bringup --cmake-args -DBUILD_TESTING=OFF "-DCMAKE_PREFIX_PATH=$RUNTIME/sdk_install;/opt/ros/humble"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/generate_config.py"
