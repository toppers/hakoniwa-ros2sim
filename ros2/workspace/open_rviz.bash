#!/bin/bash

source install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch hakoniwa_turtlebot3_description display.launch.py