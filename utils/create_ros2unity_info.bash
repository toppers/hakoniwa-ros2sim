#!/bin/bash

if [ -z "$CORE_IPADDR" ]
then
	echo "ERROR: env variable CORE_IPADDR is not set"
	exit 1
fi

if [ $# -ne 3 ] && [ $# -ne 4 ]
then
	echo "Usage: $0 {ros1|ros2} <unity project root> <setting folder> [msg|json|config|code]" 
	exit 1
fi

PHASE="all"
if [ $# -eq 4 ]
then
	PHASE="${4}"
fi

ROS_VERSION=${1}
UNITY_PRJ_DIR=${2}
SETTING_FOLDER=${3}
ROS_JSON_DIR=${ROS_VERSION}/ros_json

if [ "${ROS_VERSION}" = "ros1" ]
then
	export ROS_IPADDR=127.0.0.1
elif [ "${ROS_VERSION}" = "ros2" ]
then
	NETWORK_INTERFACE=$(route | grep '^default' | grep -o '[^ ]*$' | tr -d '\n')
	IP_ADDR=$(ifconfig "${NETWORK_INTERFACE}" | grep netmask | awk '{print $2}')
	export ROS_IPADDR=$IP_ADDR
else
  exit 1
fi

if [ -z "${ROS_UNITY_IPADDR}" ]
then
	export ROS_UNITY_IPADDR=${ROS_IPADDR}
fi

if [ -d "${ROS_JSON_DIR}" ]
then
	:
else
	echo "ERROR: Can not found folder: ${ROS_JSON_DIR}"
	exit 1
fi

if [ -d "${UNITY_PRJ_DIR}" ]
then
	:
else
	echo "ERROR: Can not found folder: ${UNITY_PRJ_DIR}"
	exit 1
fi

if [ -d "${SETTING_FOLDER}" ]
then
	:
else
	echo "ERROR: Can not found folder: ${SETTING_FOLDER}"
	exit 1
fi

if [ -f "${SETTING_FOLDER}/${ROS_VERSION}_search_file_path.txt" ]
then
	:
else
	echo "ERROR: Can not found folder: ${SETTING_FOLDER}/${ROS_VERSION}_search_file_path.txt"
	exit 1
fi


PKG_NAME=$(echo "${SETTING_FOLDER}" | sed -e 's/\// /g' | awk '{print $NF}')
UNITY_DST_DIR=${UNITY_PRJ_DIR}/Assets/Scripts/Hakoniwa/PluggableAsset/Communication

if [ "${PHASE}" = "all" ] || [ "${PHASE}" = "msg" ]
then
	echo "###Phase1(msg): Parsing ros_msgs from ${SETTING_FOLDER}/RosTopics.json"
	bash utils/create_all_ros_msgs.bash "${SETTING_FOLDER}"/"${ROS_VERSION}"_search_file_path.txt "${SETTING_FOLDER}"/RosTopics.json | tee "${SETTING_FOLDER}"/ros_msgs.txt
	if [ "${ROS_VERSION}" = "ros1" ]
	then
		echo "builtin_interfaces/Time" | tee -a "${SETTING_FOLDER}"/ros_msgs.txt
	fi
	echo "###Phase1(msg): Success"
fi

if [ "${PHASE}" = "all" ] || [ "${PHASE}" = "json" ]
then
	echo "###Phase2(json): Creating ros_json from ${SETTING_FOLDER}/ros_msgs.txt"
	bash utils/convert_rosmsg2json.bash "${SETTING_FOLDER}"/"${ROS_VERSION}"_search_file_path.txt "${SETTING_FOLDER}"/ros_msgs.txt "${ROS_JSON_DIR}"
	echo "###Phase2(json): Success"
fi

if [ "${PHASE}" = "all" ] || [ "${PHASE}" = "config" ]
then
	echo "###Phase3(config): Creating core_config"
	bash utils/core_config/all.bash "${ROS_VERSION}" "${UNITY_PRJ_DIR}" "${SETTING_FOLDER}"
	echo "###Phase3(json): Success"
fi

if [ "${PHASE}" = "all" ] || [ "${PHASE}" = "code" ]
then
	echo "###Phase4(code): Creating Unity Scripts"

	rm -rf input
	rm -rf output

	mkdir input
	mkdir output

	if [ -f "${SETTING_FOLDER}"/ros_msgs.txt ]
	then
		while IFS= read -r line
		do
			cp "${ROS_JSON_DIR}"/"${line}".json input/
		done < "${SETTING_FOLDER}"/ros_msgs.txt
		< "${SETTING_FOLDER}"/ros_msgs.txt awk -F/ '{print $1}' | sort | uniq | awk -F_msgs '{print $1}' > input/msg_pkg.txt
	fi

	cp utils/template/*.tpl input/

	bash ./utils/generate.bash ./input ./output "${PKG_NAME}"

	UPPER_PKG_NAME=$(echo "${PKG_NAME}" | tr '[:lower:]' '[:upper:]')

	UNITY_SRC_FILE1=./output/RosTopicIo.cs
	UNITY_SRC_FILE2=./output/RosTopicPduReaderConverter.cs
	UNITY_SRC_FILE3=./output/RosTopicPduWriterConverter.cs
	UNITY_SRC_FILE4=./output/RosTopicPduCommTypedData.cs
	UNITY_DST_DIR1=${UNITY_DST_DIR}/Method/ROS/${UPPER_PKG_NAME}/
	UNITY_DST_DIR2=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
	UNITY_DST_DIR3=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
	UNITY_DST_DIR4=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/


	function copy()
	{
		if [ -d "$2" ]
		then
			:
		else
			mkdir -p "$2"
		fi
		echo "Copying $1 to $2"
		cp -rp "$1" "$2"
	}

	copy ${UNITY_SRC_FILE1} "${UNITY_DST_DIR1}"
	copy ${UNITY_SRC_FILE2} "${UNITY_DST_DIR2}"
	copy ${UNITY_SRC_FILE3} "${UNITY_DST_DIR3}"
	copy ${UNITY_SRC_FILE4} "${UNITY_DST_DIR4}"


	for i in input/*.json
	do
		MSG_NAME=$(echo "$i" | awk -F"${IN_DIR}"/ '{print $2}' | awk -F\. '{print $1}')
		copy ./output/"${MSG_NAME}"Accessor.cs "${UNITY_DST_DIR}"/Pdu/Accessor/
	done


	rm -rf input
	rm -rf output

	echo "###Phase4(code): Success"
fi


