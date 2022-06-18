#!/bin/bash

HAKO_CPP=None
if [ $# -eq 1 -a "$1" = "cpp" ]
then
	HAKO_CPP=True
fi

CURDIR=`pwd`

cd ../..

if [ ${OS_TYPE} = "Mac" ]
then
	bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 config
else
	bash utils/create_ros2unity_info_wsl2.bash ros2 ros2/unity/tb3 settings/tb3 config
fi

cd ros2/unity
bash install.bash
if [ ${HAKO_CPP} = "True" ]
then
	bash install-cpp.bash
fi

cd ${CURDIR}
if [ -d src/ros_tcp_endpoint ]
then
	:
else
	bash build.bash
fi

