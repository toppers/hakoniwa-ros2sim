#!/bin/bash

DOCKER_IMAGE=`cat docker/image_name.txt`
DOCKER_ID=`docker ps | grep "${DOCKER_IMAGE}" | awk '{print $1}'`

docker exec -it ${DOCKER_ID} /bin/bash
