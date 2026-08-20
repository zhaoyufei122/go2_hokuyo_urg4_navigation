#!/usr/bin/env bash
set -euo pipefail

serial_port="${GO2_HOKUYO_SERIAL:-/dev/ttyACM0}"
ssh_target="${GO2_HOKUYO_SSH_TARGET:-}"
unitree_setup="${UNITREE_ROS2_SETUP:-/home/yufei/Desktop/unitree_ros2/setup.sh}"

# The URG-04LX physically scans 240°: 120° left + 120° right of its forward
# axis (±2.0944 rad). The rear 120° is a hardware blind zone; mount the lidar
# with that blind zone facing the robot arm so the arm never appears in scans.
# If the arm still shows up inside the ±120° window, tighten the range with
# GO2_HOKUYO_ANGLE_MIN/MAX (radians, 0 = lidar forward).
angle_min="${GO2_HOKUYO_ANGLE_MIN:--2.0944}"
angle_max="${GO2_HOKUYO_ANGLE_MAX:-2.0944}"

urg_args=(
    -r "scan:=/scan_raw"
    -p "serial_port:=${serial_port}"
    -p "laser_frame_id:=laser"
    -p "angle_min:=${angle_min}"
    -p "angle_max:=${angle_max}"
)

ros_distro="${GO2_HOKUYO_ROS_DISTRO:-foxy}"
remote_ws="${GO2_HOKUYO_WS:-/home/unitree/hokuyo_ws}"

if [[ -n "${ssh_target}" ]]; then
    # Remote mode: Hokuyo is plugged into a USB port of the GO2 Jetson.
    network_interface="${GO2_HOKUYO_INTERFACE:-eth0}"
    host_key_alias="${GO2_HOKUYO_HOST_KEY_ALIAS:-go2-hokuyo-jetson}"

    dds_uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
    printf -v quoted_serial "%q" "${serial_port}"
    printf -v quoted_dds_uri "%q" "${dds_uri}"
    # Add ~1 s to compensate the ssh + sudo latency before date runs remotely.
    printf -v quoted_time "%q" "@$(( $(date +%s) + 1 ))"
    printf -v quoted_angle_min "%q" "${angle_min}"
    printf -v quoted_angle_max "%q" "${angle_max}"

    # The Jetson has no RTC battery, so its clock is wrong after every boot.
    # Sync it to this computer before starting the driver, otherwise scan
    # timestamps fall outside the TF cache and slam_toolbox/RViz drop them.
    remote_command="sudo date -s ${quoted_time} && source /opt/ros/${ros_distro}/setup.bash && source ${remote_ws}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=${quoted_dds_uri} && exec ros2 run urg_node urg_node_driver --ros-args -r scan:=/scan_raw -p serial_port:=${quoted_serial} -p laser_frame_id:=laser -p angle_min:=${quoted_angle_min} -p angle_max:=${quoted_angle_max}"

    printf 'Starting Hokuyo URG-04LX-UG01 on %s (port %s). Keep this terminal open; Ctrl-C stops the driver.\n' "${ssh_target}" "${serial_port}"
    printf 'Syncing Jetson clock first; you may be asked for the sudo password on the Jetson.\n'
    exec ssh -tt \
        -o "HostKeyAlias=${host_key_alias}" \
        -o StrictHostKeyChecking=accept-new \
        "${ssh_target}" \
        "${remote_command}"
fi

# Local mode: Hokuyo is plugged into this computer.
if [[ ! -e "${serial_port}" ]]; then
    printf 'Hokuyo serial device not found: %s\n' "${serial_port}" >&2
    printf 'Plug in the URG-04LX-UG01 over USB, then check: ls /dev/ttyACM*\n' >&2
    printf 'Or override the port: GO2_HOKUYO_SERIAL=/dev/ttyACM1 %s\n' "$0" >&2
    printf 'To run on the GO2 Jetson instead: GO2_HOKUYO_SSH_TARGET=unitree@192.168.123.18 %s\n' "$0" >&2
    exit 1
fi
if [[ ! -f "${unitree_setup}" ]]; then
    printf 'Unitree setup not found: %s\n' "${unitree_setup}" >&2
    exit 1
fi

set +u
source "${unitree_setup}"
set -u

printf 'Starting Hokuyo URG-04LX-UG01 on %s. Keep this terminal open; Ctrl-C stops the driver.\n' "${serial_port}"
exec ros2 run urg_node urg_node_driver --ros-args "${urg_args[@]}"
