#!/bin/bash

DOCKER_IMAGE=`cat docker/image_name.txt`
DOCKER_FILE=docker/Dockerfile
docker build -t ${DOCKER_IMAGE} -f ${DOCKER_FILE} .

