#!/bin/bash

IMAGE_NAME=`cat docker/image_name.txt`
IMAGE_TAG=`cat appendix/latest_version.txt`
DOCKER_IMAGE=${IMAGE_NAME}:${IMAGE_TAG}

docker pull ${DOCKER_IMAGE}
