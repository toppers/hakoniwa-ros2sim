#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2-builder:v1.0.0
DOCKER_ID=`sudo docker ps | grep "${DOCKER_IMAGE}" | awk '{print $1}'`

sudo docker exec -it ${DOCKER_ID} /bin/bash
