#!/bin/bash

if [ $# -ne 2 ]
then
    echo "Usage: $0 <install_dir> <pkg/msg>"
    exit 1
fi

ROS_INSTALL_DIR=${1}
PKG_MSG=${2}
PKG_NAME=`echo ${PKG_MSG} | awk -F/ '{print $1}'`
MSG_NAME=`echo ${PKG_MSG} | awk -F/ '{print $2}'`

if [ -f ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${MSG_NAME}.msg ]
then
    :
else
    exit 1
fi

cat ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${MSG_NAME}.msg | grep -v "^#" | egrep -v  "^[\ \t]+\#" | grep -v "^$" | awk '{print $1}' | sort |uniq > .tmp

for i in `cat .tmp`
do
    COUNT=`echo $i | awk -F/ '{print NF}'`
    if [ $COUNT -eq 2 ]
    then
        echo $i | awk -F[ '{print $1}'
    else
        type=`echo ${i} | awk -F[ '{print $1}'`
        if [ -f ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${type}.msg ]
        then
            echo ${PKG_NAME}/${type}
        else
        	STR=`find ${ROS_INSTALL_DIR} -name ${type}.msg`
        	if [ -z ${STR} ]
        	then
        		:
        	else
				TMP_PKG_NAME=`echo $STR | awk -F/ '{print $(NF-2)}'`
	            echo ${TMP_PKG_NAME}/${type}
        	fi
        fi
    fi
done

rm -f .tmp

exit 0
