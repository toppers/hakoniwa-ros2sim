#!/bin/bash

export CORE_IPADDR=`cat /etc/resolv.conf  | grep nameserver | awk '{print $NF}'`

cat settings/tb3/ros2_search_file_path.txt | grep -v ros2sim > tmp.txt
CURDIR=`pwd`
echo "${CURDIR}/ros2/workspace/src" >> tmp.txt
mv tmp.txt settings/tb3/ros2_search_file_path.txt
bash utils/create_ros2unity_info.bash $*
