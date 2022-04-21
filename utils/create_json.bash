#!/bin/bash

if [ $# -ne 2 ]
then
	echo "Usage: $0 <in> <out>"
	exit 1
fi

IN_DIR=${1}
OUT_DIR=${2}

for i in `ls ${IN_DIR}/*.msg`
do
	MSG_NAME=`echo $i | awk -F\. '{print $1}'`
	echo $MSG_NAME
	python2 utils/rosmsg2json.py ${i} ${OUT_DIR}
done


