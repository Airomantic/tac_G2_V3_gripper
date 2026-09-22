#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Use the activated venv/Conda environment, or an explicitly selected Python.
if [[ -n "${PYTHON:-}" ]]; then
    PYTHON_BIN="$PYTHON"
elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
else
    echo "Error: Python was not found. Install Python 3.10 or later." >&2
    exit 1
fi

"$PYTHON_BIN" - <<'PY'
import sys

if sys.version_info < (3, 10):
    sys.exit("Error: DM_V3Gripper requires Python 3.10 or later. Select a compatible environment.")
print(f"Installing with Python: {sys.executable}")
PY

if ! "$PYTHON_BIN" -m pip --version >/dev/null 2>&1; then
    echo "Error: pip is missing from the selected Python environment. Install pip first." >&2
    exit 1
fi

# An absolute source path makes the script usable from any working directory.
"$PYTHON_BIN" -m pip install "$SCRIPT_DIR" "$@"

# Exclude the working directory so source files cannot mask a failed install.
"$PYTHON_BIN" - <<'PY'
import sys

if sys.path and sys.path[0] == "":
    sys.path.pop(0)
import DM_V3Gripper
from DM_V3Gripper import Gripper, gripper_pb2, gripper_pb2_grpc

print(f"Successfully installed DM_V3Gripper {DM_V3Gripper.__version__}: {DM_V3Gripper.__file__}")
print("Use this import in any script in the same Python environment: from DM_V3Gripper import Gripper")
PY
