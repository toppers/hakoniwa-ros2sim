#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2-builder

HAKONIWA_TOP_DIR=$(cd ../.. && pwd)
DOCKER_IMAGE=${DOCKER_IMAGE}:v1.0.0

docker run -v ${HAKONIWA_TOP_DIR}:/root/workspace/hakoniwa-ros-samples \
		-e CORE_IPADDR=127.0.0.1 \
		-e ROS_IPADDR=127.0.0.1 \
		-e OS_TYPE="Linux" \
		-it --rm --net host --name hakoniwa-ros-sim ${DOCKER_IMAGE} 
