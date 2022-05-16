#!/bin/bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

if [ $# -ne 2 ]
then
	echo "Usage: $0 <pkgname> <RoboName>"
	exit 1
fi

ROSNODE_NAME=${1}_node
ROBO_NAME=${2}
IS_EXIT="FALSE"
ROSNODE_PID="NONE"
signal_handler () {
	if [ ${ROSNODE_PID} = "NONE" ]
	then
		echo "NONE"
	else
		kill -s TERM ${ROSNODE_PID}
		sleep 1
		#ps aux | grep ${ROSNODE_NAME} | grep -v grep | awk '{print $2}' | xargs kill -s TERM
		#sleep 1
		echo "Terminated!! ${ROSNODE_PID}"
		IS_EXIT="TRUE"
	fi
}

trap signal_handler INT TERM

#/root/workspace/hakoniwa-ros-sim/ros2/workspace/tmp.bash &
/opt/ros/foxy/bin/ros2 run ${1} ${ROSNODE_NAME} ${ROBO_NAME} &
ROSNODE_PID=$!
echo "ROSNODE_PID=$ROSNODE_PID"


while [ 1 ]
do
	if [ ${IS_EXIT} = "TRUE" ]
	then
		exit 0
	fi
	sleep 1
done
