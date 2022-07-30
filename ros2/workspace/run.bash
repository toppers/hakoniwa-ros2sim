#!/bin/bash

source install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
if [ $# -ne 1 -a $# -ne 2 -a $# -ne 3 ]
then
	echo "Usage: $0 <pkgname> [args]"
	exit 1
fi
ros2 run ${1} ${1}_node ${2} ${3}
