#!/bin/bash

if [ $# -ne 3 ]
then
    echo "Usage: $0 <search_path file> <ros msg list file> <dst_dir>"
    exit 1
fi

SEARCH_PATH_FILE=${1}
ROS_MSG_LIST_FILE=${2}
DST_DIR=${3}

ROS_VERSION=`echo ${DST_DIR} | awk -F/ '{print $1}'`

RET_VALUE="NONE"
function get_filepath()
{
    RET_VALUE="NONE"
    local_pkg_msg=${1}
    pkg_name=`echo ${1} | awk -F/ '{print $1}'`
    msg_name=`echo ${1} | awk -F/ '{print $2}'`
    for path in `cat ${SEARCH_PATH_FILE}`
    do
        if [ -f ${path}/${pkg_name}/msg/${msg_name}.msg ]
        then
            RET_VALUE=${path}/${pkg_name}/msg/${msg_name}.msg
            break
        fi
    done
}

function create_ros_json_file()
{
    local_pkg_msg=${1}
    echo "#### Creating ${local_pkg_msg} ####"
    filepath=${2}
    pkg_name=`echo ${1} | awk -F/ '{print $1}'`
    msg_name=`echo ${1} | awk -F/ '{print $2}'`
    if [ -d ${DST_DIR}/${pkg_name} ]
    then
        :
    else
        mkdir -p ${DST_DIR}/${pkg_name}
    fi
    python2 utils/rosmsg2json.py ${filepath} ${DST_DIR}/${pkg_name}
}

NEED_CREATE="TRUE"
function check_version()
{
    pkg_name=`echo ${1} | awk -F/ '{print $1}'`
    msg_name=`echo ${1} | awk -F/ '{print $2}'`
	NEED_CREATE="TRUE"
	if [ "${ROS_VERSION}" = "ros1" ]
	then
		if [ ${pkg_name} = "builtin_interfaces" ]
		then
			NEED_CREATE="FALSE"
		fi
	fi
}

for pkg_msg in `cat ${ROS_MSG_LIST_FILE}`
do
	check_version ${pkg_msg}
	if [ "${NEED_CREATE}" = "TRUE" ]
	then
	    get_filepath ${pkg_msg}
	    if [ ${RET_VALUE} != "NONE" ]
	    then
	        create_ros_json_file ${pkg_msg} ${RET_VALUE}
	    else
	        echo "ERROR: can not found pkg_msg=${pkg_msg}"
	        exit 1
	    fi
	fi
done

