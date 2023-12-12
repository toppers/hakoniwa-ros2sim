#!/bin/bash

# Unity
#cp config/hakoniwa_path.json hakoniwa-unity-tb3model/plugin/plugin-srcs/hakoniwa_path.json 

# ROS2
# cp -rp workspace/dev/tb3 hakoniwa-ros2pdu/workspace/src/
# mv hakoniwa-ros2pdu/workspace/src/tb3/build-tb3.bash hakoniwa-ros2pdu/workspace/

# mros2-posix pico_msgs
ORG_DIR=`pwd`
cp -rp hakoniwa-ros2pdu/template/mros2-patch/pico_msgs  hakoniwa-ros2pdu/mros2-posix/workspace/custom_msgs/
cd hakoniwa-ros2pdu/mros2-posix/workspace
python3 ../mros2/mros2_header_generator/header_generator.py pico_msgs/msg/LightSensor.msg
cd ${ORG_DIR}

# mros2proxy
cp config/custom.json hakoniwa-ros2pdu/config/
cp config/ros_msgs.txt hakoniwa-ros2pdu/config/

