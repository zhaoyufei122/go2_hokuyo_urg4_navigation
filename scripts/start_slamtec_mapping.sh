#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unitree_setup="${UNITREE_ROS2_SETUP:-/home/yufei/Desktop/unitree_ros2/setup.sh}"
workspace_setup="${workspace_root}/install/setup.bash"

if [[ ! -f "${unitree_setup}" ]]; then
    printf 'Unitree setup not found: %s\n' "${unitree_setup}" >&2
    exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
    printf 'Workspace is not built. Run: cd %s && colcon build --symlink-install\n' "${workspace_root}" >&2
    exit 1
fi

set +u
source "${unitree_setup}"
source "${workspace_setup}"
set -u

exec ros2 launch go2_base_nav slamtec_mapping.launch.py "$@"
