#!/usr/bin/env zsh
set -e

WORKSPACE_DIR=${0:A:h}
source /opt/ros/humble/setup.zsh

if [[ ! -f ${WORKSPACE_DIR}/install/setup.zsh ]]; then
  print -u2 "Workspace is not built: ${WORKSPACE_DIR}"
  print -u2 "Run: cd ${WORKSPACE_DIR} && colcon build --symlink-install"
  exit 1
fi

source ${WORKSPACE_DIR}/install/setup.zsh
exec ros2 run standard_motion_recorder motion_control.zsh "$@"
