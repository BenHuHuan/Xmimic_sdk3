#!/usr/bin/env bash
TG_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# 1. Source ROS2 (Auto-detect Jazzy, Humble, or vendor xos)
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
    ROS_VER=24
    ROS_NAME=jazzy
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    ROS_VER=22
    ROS_NAME=humble
elif [[ -f /home/ubuntu/xos/setup.bash ]]; then
    source /home/ubuntu/xos/setup.bash
    ROS_VER=24
    ROS_NAME=jazzy
else
    echo "Warning: No standard ROS2 installation found (/opt/ros/jazzy, /opt/ros/humble, or /home/ubuntu/xos)" >&2
fi

export XMIGCS_ROOT="$TG_ROOT/sdk"

# Add vendor unpacked ROS packages if present
if [[ -d "$TG_ROOT/vendor/opt/ros/$ROS_NAME" ]]; then
    export PYTHONPATH="$TG_ROOT/vendor/opt/ros/$ROS_NAME/local/lib/python3.10/dist-packages:$TG_ROOT/vendor/opt/ros/$ROS_NAME/lib/python3.12/site-packages:$XMIGCS_ROOT${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$TG_ROOT/vendor/opt/ros/$ROS_NAME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export AMENT_PREFIX_PATH="$TG_ROOT/vendor/opt/ros/$ROS_NAME${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
else
    export PYTHONPATH="$XMIGCS_ROOT${PYTHONPATH:+:$PYTHONPATH}"
fi

export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [[ -f "$TG_ROOT/.venv/bin/activate" ]]; then
    source "$TG_ROOT/.venv/bin/activate"
    export VIRTUAL_ENV="$TG_ROOT/.venv"
    export PATH="$VIRTUAL_ENV/bin:$PATH"
fi
hash -r 2>/dev/null || true
