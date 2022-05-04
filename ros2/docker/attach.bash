#!/bin/bash

DOCKER_IMAGE=`cat image_name.txt`
DOCKER_ID=`sudo docker ps | grep "${DOCKER_IMAGE}" | awk '{print $1}'`

sudo docker exec -it ${DOCKER_ID} /bin/bash
