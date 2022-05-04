#!/bin/bash

DOCKER_IMAGE=`cat image_name.txt`
DOCKER_FILE=Dockerfile
sudo docker build -t ${DOCKER_IMAGE} -f ${DOCKER_FILE} .

