#!/usr/bin/env bash
# ROS 2 Jazzy requires system Python 3.12, not the sensor SDK's Python 3.10 venv.
set -euo pipefail

GRIPPER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GRIPPER_PYTHON="$GRIPPER_DIR/.venv-ros/bin/python"

if [[ "${1:-}" == setup ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$GRIPPER_DIR/.venv-ros"
    "$GRIPPER_PYTHON" -m pip install "$GRIPPER_DIR/G3_client"
    exit 0
fi

if [[ ! -x "$GRIPPER_PYTHON" ]]; then
    echo "请先执行：$0 setup" >&2
    exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "未找到 /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

# Preserve ROS_DOMAIN_ID/RMW network settings while removing unrelated overlays.
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    bash --noprofile --norc -c \
    'source /opt/ros/jazzy/setup.bash; exec "$@"' _ \
    "$GRIPPER_PYTHON" "$GRIPPER_DIR/teleop_gripper.py" "$@"
