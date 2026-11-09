#!/usr/bin/env bash
TG_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
export XMIGCS_ROOT="$TG_ROOT/sdk"
export PYTHONPATH="$TG_ROOT/vendor/opt/ros/humble/local/lib/python3.10/dist-packages:$XMIGCS_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$TG_ROOT/vendor/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$TG_ROOT/vendor/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source "$TG_ROOT/.venv/bin/activate"
