#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unitree_setup="${UNITREE_ROS2_SETUP:-/home/yufei/Desktop/unitree_ros2/setup.sh}"
workspace_setup="${workspace_root}/install/setup.bash"
map_yaml="${GO2_HOKUYO_MAP:-${workspace_root}/maps/hokuyo_room.yaml}"

if [[ $# -gt 0 && "$1" != *":="* ]]; then
    map_yaml="$1"
    shift
fi
if [[ ! -f "${map_yaml}" ]]; then
    printf 'Hokuyo map not found: %s\n' "${map_yaml}" >&2
    printf 'Build and save the map before starting navigation.\n' >&2
    exit 1
fi
if [[ ! -f "${unitree_setup}" ]]; then
    printf 'Unitree setup not found: %s\n' "${unitree_setup}" >&2
    exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
    printf 'Workspace is not built. Run: cd %s && colcon build --symlink-install\n' "${workspace_root}" >&2
    exit 1
fi

if [[ "${map_yaml}" != /* ]]; then
    map_yaml="$(realpath "${map_yaml}")"
fi

# slam_posegraph 也必须转绝对路径：ros2 launch 起节点的 CWD 不保证是
# 脚本所在目录，相对路径会让 slam_toolbox 找不到文件（日志报
# "Failed to open requested file"），posegraph 加载失败 = 拿空图做
# localization = 看起来像重新建图（2026-08-24 实测踩坑）。
# 同时去掉用户误带的 .posegraph 后缀（slam_toolbox 自己会补）。
args=()
for arg in "$@"; do
    if [[ "${arg}" == slam_posegraph:=* ]]; then
        pg="${arg#slam_posegraph:=}"
        pg="${pg%.posegraph}"
        if [[ -n "${pg}" && "${pg}" != /* ]]; then
            pg="$(realpath "${pg}")"
        fi
        if [[ -n "${pg}" && ! -f "${pg}.posegraph" ]]; then
            printf 'slam_posegraph not found: %s.posegraph\n' "${pg}" >&2
            exit 1
        fi
        arg="slam_posegraph:=${pg}"
    fi
    args+=("${arg}")
done
set -- "${args[@]}"

set +u
source "${unitree_setup}"
source "${workspace_setup}"
set -u

exec ros2 launch go2_base_nav hokuyo_navigation.launch.py map:="${map_yaml}" "$@"
