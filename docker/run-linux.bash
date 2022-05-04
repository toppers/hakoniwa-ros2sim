#!/bin/bash

HAKONIWA_TOP_DIR=$(cd ../.. && pwd)
DOCKER_IMAGE=`cat image_name.txt`

docker run -v ${HAKONIWA_TOP_DIR}:/root/workspace/hakoniwa-ros-samples \
		-e CORE_IPADDR=127.0.0.1 \
		-e ROS_IPADDR=127.0.0.1 \
		-e OS_TYPE="Linux" \
		-it --rm --net host --name hakoniwa-ros-sim ${DOCKER_IMAGE} 
