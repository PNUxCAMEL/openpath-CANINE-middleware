#!/bin/bash


# 현재 스크립트의 디렉토리 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
JAZZY_SETUP="/opt/ros/jazzy/setup.bash"
SCRIPT_SETUP="${PARENT_DIR}/ros_pkg/install/setup.bash"
# ROS 2 jazzy 확인
if [ -f "$JAZZY_SETUP" ]; then
    echo "ROS 2 Jazzy found. Sourcing $JAZZY_SETUP"
    source "$JAZZY_SETUP"
else
    echo "Error: ROS 2 Jazzy is not found in /opt/ros."
    exit 1
fi

cd "${PARENT_DIR}/ros_pkg"

colcon build --packages-select canine_msgs_v2 --symlink-install

echo "Build Finish"