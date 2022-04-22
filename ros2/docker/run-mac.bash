#!/bin/bash
if [ $# -ne 1 ]
then
	echo "Usage: $0 <eth>"
	exit 1
fi
ETHER=${1}
IPADDR=`ifconfig | grep -A1 ${ETHER} | grep netmask | awk '{print $2}'`
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

docker run -v ${HAKONIWA_TOP_DIR}:/root/workspace/hakoniwa-ros-samples \
	-it --rm --ip ${IPADDR} -p 10000:10000 \
	-e CORE_IPADDR=${IPADDR} \
	-e ROS_UNITY_IPADDR=${IPADDR} \
	-e OS_TYPE="Mac" \
	--name hakoniwa-ros-sim ${DOCKER_IMAGE} 
