#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
ROOT_DIR=$(pwd)

# Detect ROS 2 distribution
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    ROS_LIB="24"
    ROS_NAME="jazzy"
    PYTHON_TARGET="3.12"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
    ROS_LIB="22"
    ROS_NAME="humble"
    PYTHON_TARGET="3.10"
elif [[ -f /home/ubuntu/xos/setup.bash ]]; then
    ROS_SETUP="/home/ubuntu/xos/setup.bash"
    ROS_LIB="24"
    ROS_NAME="jazzy"
    PYTHON_TARGET="3.12"
else
    echo "Error: ROS2 (Jazzy, Humble, or vendor xos) is required." >&2
    exit 1
fi

echo "Detected ROS environment: $ROS_NAME ($ROS_SETUP), target Python $PYTHON_TARGET"

[[ $(uname -m) == x86_64 ]] || { echo "Bundled motor conversion libraries require x86_64"; exit 1; }
command -v dpkg-deb >/dev/null || { echo "dpkg-deb is required to unpack vendor ROS messages."; exit 1; }

# Unpack matching vendor bodyctrl deb package
mkdir -p vendor
for deb in sdk/lib/$ROS_LIB/*.deb; do
    if [[ -f "$deb" ]]; then
        echo "Unpacking $deb into vendor/..."
        dpkg-deb -x "$deb" vendor
    fi
done

relocate_venv() {
  local venv_dir="$ROOT_DIR/.venv"
  [[ -d "$venv_dir/bin" ]] || return 0
  if [[ -f "$venv_dir/bin/activate" ]]; then
    sed -i "s|^VIRTUAL_ENV=.*|VIRTUAL_ENV='$venv_dir'|" "$venv_dir/bin/activate"
  fi
  for f in "$venv_dir/bin/"*; do
    [[ -f "$f" && ! -L "$f" ]] || continue
    if head -c 2 "$f" | grep -q "^#!"; then
      sed -i "1s|^#!/.*/\.venv/bin/python|#!$venv_dir/bin/python|" "$f" 2>/dev/null || true
    fi
  done
}

venv_ready() {
  [[ -x .venv/bin/python ]] || return 1
  .venv/bin/python -c "import sys; target = tuple(map(int, '$PYTHON_TARGET'.split('.'))); assert sys.version_info[:2] == target" >/dev/null 2>&1 || return 1
  relocate_venv
  (
    set +u
    source ./env.sh
    python -c "import rclpy, sptlib_python, mujoco, onnxruntime" >/dev/null 2>&1
  )
}

if ! venv_ready; then
  echo "Setting up Python $PYTHON_TARGET virtual environment..."
  # Clean up existing incompatible venv if wrong Python version
  if [[ -d .venv ]]; then
    if ! .venv/bin/python -c "import sys; target = tuple(map(int, '$PYTHON_TARGET'.split('.'))); assert sys.version_info[:2] == target" >/dev/null 2>&1; then
      echo "Existing .venv Python version mismatch. Re-creating .venv for Python $PYTHON_TARGET..."
      rm -rf .venv
    fi
  fi

  UV=${UV:-$(command -v uv || true)}
  [[ -n "$UV" ]] || { [[ ! -x "$HOME/.local/bin/uv" ]] || UV="$HOME/.local/bin/uv"; }
  if [[ -z "$UV" ]]; then
    if command -v curl >/dev/null; then
      bootstrap=$(mktemp)
      trap 'rm -f "$bootstrap"' EXIT
      if curl --fail --location --connect-timeout 5 --retry 2 https://astral.sh/uv/install.sh -o "$bootstrap"; then
        UV_NO_MODIFY_PATH=1 sh "$bootstrap"
        UV="$HOME/.local/bin/uv"
      fi
    fi
  fi

  if [[ -n "$UV" && -x "$UV" ]]; then
    if [[ ! -x .venv/bin/python ]]; then
      "$UV" venv --python "$PYTHON_TARGET" .venv
    fi
    "$UV" pip install --python .venv/bin/python -r requirements.txt
    "$UV" pip install --python .venv/bin/python sdk/lib/$ROS_LIB/sptlib_python*.whl
  else
    # Fallback to python3 -m venv if uv is not available
    echo "uv not found; attempting system python3 venv..."
    PYTHON_CMD="python$PYTHON_TARGET"
    command -v "$PYTHON_CMD" >/dev/null || PYTHON_CMD="python3"
    if [[ ! -x .venv/bin/python ]]; then
      "$PYTHON_CMD" -m venv .venv
    fi
    .venv/bin/pip install --upgrade pip
    .venv/bin/pip install -r requirements.txt
    .venv/bin/pip install sdk/lib/$ROS_LIB/sptlib_python*.whl
  fi
  relocate_venv
fi

set +u
source ./env.sh
python -c "import rclpy, sptlib_python, mujoco, onnxruntime; print('Runtime imports OK')"
python sdk/local_setup/check_sdk_a_hardware.py
printf "Installed successfully for ROS2 $ROS_NAME. Run: ./xrobot3 run --interface lo (simulation) or ./xrobot3 run --target real --mode onboard
"
