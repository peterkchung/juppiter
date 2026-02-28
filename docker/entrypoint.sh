#!/bin/bash
# About: Container entrypoint that sources the ROS 2 workspace overlay.
set -e

source /opt/ros/kilted/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
