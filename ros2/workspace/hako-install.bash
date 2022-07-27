#!/bin/bash

HAKO_CPP=None
if [ $# -eq 1 -a "$1" = "cpp" ]
then
	HAKO_CPP=True
fi

OPT=config
if [ $# -eq 2 -a "$1" = "opt" ]
then
	OPT=${2}
fi

CURDIR=`pwd`

cd ../..

if [ ${OS_TYPE} = "Mac" ]
then
	bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 ${OPT}
else
	bash utils/create_ros2unity_info_wsl2.bash ros2 ros2/unity/tb3 settings/tb3 ${OPT}
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

