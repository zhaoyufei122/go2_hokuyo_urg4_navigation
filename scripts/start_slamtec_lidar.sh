#!/usr/bin/env bash
set -euo pipefail

ssh_target="${GO2_SLAMTEC_SSH_TARGET:-unitree@192.168.123.18}"
serial_port="${GO2_SLAMTEC_SERIAL:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_9e2b18dfb98eb4408d1d79d42c29a074-if00-port0}"
network_interface="${GO2_SLAMTEC_INTERFACE:-enP8p1s0}"
host_key_alias="${GO2_SLAMTEC_HOST_KEY_ALIAS:-go2-slamtec-jetson}"

dds_uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
printf -v quoted_serial "%q" "${serial_port}"
printf -v quoted_dds_uri "%q" "${dds_uri}"

remote_command="source /opt/ros/humble/setup.bash && source /home/unitree/demo_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=${quoted_dds_uri} && exec ros2 launch sllidar_ros2 sllidar_a3_launch.py serial_port:=${quoted_serial}"

printf 'Starting Slamtec A3 on %s. Keep this terminal open; Ctrl-C stops the motor.\n' "${ssh_target}"
exec ssh -tt \
    -o "HostKeyAlias=${host_key_alias}" \
    -o StrictHostKeyChecking=accept-new \
    "${ssh_target}" \
    "${remote_command}"
