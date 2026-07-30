#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${CHAPTER09_INSTALL}/setup.bash" ]]; then
  source "${CHAPTER09_INSTALL}/setup.bash"
fi

if [[ -f "${SORTING_DEMO_INSTALL}/setup.bash" ]]; then
  source "${SORTING_DEMO_INSTALL}/setup.bash"
fi

chapter_setup="${CHAPTER_WS}/${ROS_ACTIVE_CHAPTER}/install/setup.bash"
if [[ -f "${chapter_setup}" ]]; then
  source "${chapter_setup}"
fi

local_sorting_setup="/workspaces/sorting_ws/install/setup.bash"
if [[ -f "${local_sorting_setup}" ]]; then
  source "${local_sorting_setup}"
fi

exec "$@"
