#!/usr/bin/env bash
# Dev-only: MuJoCo 4.0 for the simulator (./xrobot3 run --interface lo --physics dev).
# PyPI MuJoCo 4.0 needs Python >= 3.11, but ROS 2 Humble rclpy is Python 3.10.
# The official 4.0 C library is built with clang/libc++, so the core bindings cannot be
# compiled against it with GCC (std::string layout mismatch -> segfault on MjSpec names).
# Build both the C library and the bindings with the system GCC instead.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PYTHON:-/usr/bin/python3.10}
JOBS=${JOBS:-8}
WHEEL_DIR="$ROOT/vendor/wheels"
TAG=$("$PY" -c 'import sys;print(f"cp{sys.version_info[0]}{sys.version_info[1]}")')
WHEEL=$(ls "$WHEEL_DIR"/mujoco_core-4.0.0-"$TAG"-*.whl 2>/dev/null | head -1 || true)
if [[ -z "$WHEEL" ]]; then
  BUILD=$(mktemp -d)
  trap 'rm -rf "$BUILD"' EXIT
  cd "$BUILD"
  git clone -q --depth 1 --branch 4.0.0 https://github.com/google-deepmind/mujoco.git src
  cmake -S src -B src/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$BUILD/lib" \
    -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_BUILD_SIMULATE=OFF \
    -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_WITH_USD=OFF
  nice -n 15 cmake --build src/build -j"$JOBS"
  cmake --install src/build
  mkdir -p "$BUILD/lib/bin/mujoco_plugin"
  cp src/build/lib/mujoco_plugin/*.so "$BUILD/lib/bin/mujoco_plugin/" 2>/dev/null || true
  curl -sSLO https://files.pythonhosted.org/packages/source/m/mujoco-core/mujoco_core-4.0.0.tar.gz
  tar xzf mujoco_core-4.0.0.tar.gz
  sed -i 's/requires-python = ">=3.11"/requires-python = ">=3.10"/' mujoco_core-4.0.0/pyproject.toml
  MUJOCO_PATH="$BUILD/lib" MUJOCO_PLUGIN_PATH="$BUILD/lib/bin/mujoco_plugin" CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" \
    nice -n 15 uv build --wheel --python "$PY" -o "$WHEEL_DIR" mujoco_core-4.0.0
  WHEEL=$(ls "$WHEEL_DIR"/mujoco_core-4.0.0-"$TAG"-*.whl | head -1)
fi
uv venv -q --python "$PY" "$ROOT/.venv-mj4"
grep -v '^mujoco' "$ROOT/requirements.txt" > "$ROOT/.venv-mj4/requirements.txt"
uv pip install -q --python "$ROOT/.venv-mj4/bin/python" -r "$ROOT/.venv-mj4/requirements.txt" "$WHEEL" glfw pyopengl
# Same bundled sptlib wheel as the stable environment (ankle parallel mechanism).
SPT=$(ls "$ROOT"/sdk/lib/22/sptlib_python-*.whl 2>/dev/null | head -1 || true)
[[ -n "$SPT" ]] && uv pip install -q --python "$ROOT/.venv-mj4/bin/python" "$SPT"
"$ROOT/.venv-mj4/bin/python" -c 'import mujoco;s=mujoco.MjSpec();print("MuJoCo",mujoco.__version__,"ready in .venv-mj4")'
