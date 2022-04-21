#!/bin/bash

CURDIR=`pwd`

cd ../..

if [ ${OS_TYPE} = "Mac" ]
then
	bash utils/install.bash ros2 ros2/unity/tb3 settings/tb3 config
else
	bash utils/wsl2_install.bash ros2 ros2/unity/tb3 settings/tb3 config
fi

cd ros2/unity
bash install.bash

cd ${CURDIR}
if [ -d src/ros_tcp_endpoint ]
then
	:
else
	bash build.bash
fi

