#!/bin/bash

if [ $# -ne 2 ]
then
    echo "Usage: $0 <search path file> <RosTopics.json>"
    exit 1
fi

SEARCH_PATH_FILE=${1}
ROS_TOPIC_FILE=${2}
grep topic_type_name ${ROS_TOPIC_FILE}  | awk '{print $2}' | awk -F\" '{print $2}' | sort | uniq > tmp.txt

bash utils/show_all_msgs.bash ${SEARCH_PATH_FILE} tmp.txt
rm -f tmp.txt