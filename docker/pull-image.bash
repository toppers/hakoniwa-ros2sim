#!/bin/bash

DOCKER_IMAGE=`cat image_name.txt`

sudo service docker start
sudo docker pull ${DOCKER_IMAGE}
