#!/bin/bash

if [ $# -ne 2 ]
then
	echo "Usage: $0 <search path file> <ros_msg_list_file>"
	exit 1
fi

SEARCH_PATH_FILE=${1}
ROS_MSG_LIST_FILE=${2}

rm -f _msgs.txt
for i in `cat ${ROS_MSG_LIST_FILE}`
do
	bash utils/parse_msg_recursive.bash ${SEARCH_PATH_FILE} ${i} >> _msgs.txt
done
cat _msgs.txt | sort | uniq 
rm -f _msgs.txt

