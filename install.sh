#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Both packages must be installed with the same Python interpreter.
if [[ -n "${PYTHON:-}" ]]; then
    PYTHON_BIN="$PYTHON"
elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
else
    echo "错误：未找到 Python，请先准备 Python 3.10～3.12 环境。" >&2
    exit 1
fi

# Resolve the interpreter before changing directories and check the shared range.
PYTHON_BIN="$("$PYTHON_BIN" - <<'PY'
import sys

if not (3, 10) <= sys.version_info < (3, 13):
    sys.exit("错误：同时安装两个包需要 Python 3.10～3.12，请切换 Python 环境。")
print(sys.executable)
PY
)"

if ! "$PYTHON_BIN" -m pip --version >/dev/null 2>&1; then
    echo "错误：所选 Python 环境缺少 pip，请先为该环境安装 pip。" >&2
    exit 1
fi

echo "安装使用的 Python：$PYTHON_BIN"

echo "[1/2] 安装 dmrobotics SDK..."
(
    cd "$SCRIPT_DIR/SDK_Publish_V1.2.13.1"
    "$PYTHON_BIN" -m pip install . "$@"
)

echo "[2/2] 安装 DM_V3Gripper..."
(
    cd "$SCRIPT_DIR/G3_client"
    PYTHON="$PYTHON_BIN" ./install.sh "$@"
)

echo "安装完成：dmrobotics 和 DM_V3Gripper 已安装到同一个 Python 环境。"
