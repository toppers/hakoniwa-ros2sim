#!/bin/bash

DOCKER_IMAGE=`cat docker/image_name.txt`

docker pull ${DOCKER_IMAGE}
