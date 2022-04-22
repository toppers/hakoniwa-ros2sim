#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2-builder
DOCKER_FILE=Dockerfile
DOCKER_TAG=v1.0.0
docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f ${DOCKER_FILE} .

