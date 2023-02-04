#!/bin/bash

if [ $# -eq 0 ]
then
	echo "Usage: $0 { cpp | opt [all|msg|json|config|code] }"
	exit 1
fi

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
if [ ${HAKO_CPP} = "True" ]
then
	bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 msg
	bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 json
	bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 config
	cd ros2/unity
	bash install-cpp.bash
else
	if [ ${OS_TYPE} = "Mac" ]
	then
		bash utils/create_ros2unity_info.bash ros2 ros2/unity/tb3 settings/tb3 ${OPT}
	else
		bash utils/create_ros2unity_info_wsl2.bash ros2 ros2/unity/tb3 settings/tb3 ${OPT}
	fi
	cd ros2/unity
	bash install.bash
fi

cd ${CURDIR}
if [ -d src/ros_tcp_endpoint ]
then
	:
else
	bash build.bash
fi

