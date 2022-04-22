#!/bin/bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <proxy file>"
	exit 1
fi

PROXY_FILE=${1}

if [ -z $CORE_IPADDR ]
then
	echo "ERROR: env variable CORE_IPADDR is not set"
	exit 1
fi
export CORE_PORTNO=50051

which hakoniwa_proxy > /dev/null
if [ $? -ne 0 ]
then
	echo "ERROR: can not found hakonwia_proxy command"
	exit 1
fi

source /opt/ros/foxy/setup.bash
source install/setup.bash

while [ 1 ]
do
        echo "############ START CONNECT #################"
        hakoniwa_proxy ./${PROXY_FILE} ${CORE_IPADDR}  ${CORE_PORTNO} sync
        echo "############ SERVER DOWN #################"
        sleep 1
done

