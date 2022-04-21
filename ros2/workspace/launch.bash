#!/bin/bash

if [ -z "${ROS_IPADDR}" ]
then
  NETWORK_INTERFACE=$(route | grep '^default' | grep -o '[^ ]*$' | tr -d '\n')
  IP_ADDR=$(ifconfig "${NETWORK_INTERFACE}" | grep netmask | awk '{print $2}')
  echo "IP_ADDR is ${IP_ADDR} (${NETWORK_INTERFACE})"
else
  IP_ADDR=${ROS_IPADDR}
  echo "IP_ADDR is ${IP_ADDR}"
fi
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="${IP_ADDR}"
