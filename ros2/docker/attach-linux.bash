#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2sim:v1.0.0
DOCKER_ID=`docker ps | grep "${DOCKER_IMAGE}" | awk '{print $1}'`

docker exec -it ${DOCKER_ID} /bin/bash
