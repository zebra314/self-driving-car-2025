#!/bin/bash
set -e

# Source the ROS 1 Noetic installation
source "/opt/ros/noetic/setup.bash"

exec "$@"