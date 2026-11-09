#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Requires ROS2 Humble / Ubuntu22.04. Install the robot-compatible ROS first.'; exit 1; }
UV=${UV:-$(command -v uv || true)}
[[ -n "$UV" ]] || { echo 'Install uv first'; exit 1; }
"$UV" venv --python /usr/bin/python3.10 .venv
"$UV" pip install --python .venv/bin/python -r requirements.txt
"$UV" pip install --python .venv/bin/python sdk/lib/22/sptlib_python*.whl
mkdir -p vendor
for deb in sdk/lib/22/*.deb; do dpkg-deb -x "$deb" vendor; done
printf 'Installed SDK and simulator in one .venv\n'
