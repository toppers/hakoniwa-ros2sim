#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2-builder

HAKONIWA_TOP_DIR=$(cd ../.. && pwd)
DOCKER_IMAGE=${DOCKER_IMAGE}:v1.0.0

sudo docker ps > /dev/null
if [ $? -ne 0 ]
then
        sudo service docker start
        echo "waiting for docker service activation.. "
        sleep 3
fi
CORE_IPADDR=`cat /etc/resolv.conf  | grep nameserver | awk '{print $NF}'`

sudo docker run -v ${HAKONIWA_TOP_DIR}:/root/workspace/hakoniwa-ros-samples \
		-it \
		-e CORE_IPADDR=${CORE_IPADDR} \
		-e OS_TYPE="wsl2" \
		--rm --net host --name hakoniwa-ros-sim ${DOCKER_IMAGE} 
