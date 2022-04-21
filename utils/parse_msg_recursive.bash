#!/bin/bash

if [ $# -ne 2 ]
then
    echo "Usage: $0 <search_path_file> <pkg/msg>"
    exit 1
fi
SEARCH_PATH_FILE=${1}
PKG_MSG=${2}

rm -f msg_types.txt

function parse_one()
{
    local_pkg_msg=`echo ${1} | awk -F[ '{print $1}'`
    rm -f entry_msg_types.txt
    for path in `cat ${SEARCH_PATH_FILE}`
    do
        bash utils/parse_msg.bash ${path} ${local_pkg_msg} > tmp.txt
        if [ $? -eq 0 ]
        then
            mv tmp.txt entry_msg_types.txt
            break;
        fi
    done
    if [ -f entry_msg_types.txt ]
    then
        :
    else
        echo "ERROR: can not found ${local_pkg_msg} on path"
        exit 1
    fi
}

function search_recursive()
{
    local_pkg_msg=`echo ${1} | awk -F[ '{print $1}'`
    while [ 1 ]
    do
        echo ${local_pkg_msg} >> msg_types.txt
        parse_one ${local_pkg_msg}
        NUM=`wc -l entry_msg_types.txt | awk '{print $1}'`
        cat entry_msg_types.txt >> msg_types.txt
        if [ ${NUM} -eq 0 ]
        then
            break;
        fi

        for e in `cat entry_msg_types.txt`
        do
            search_recursive ${e}
        done
    done
}

search_recursive ${PKG_MSG}

cat msg_types.txt | sort | uniq 
rm -f msg_types.txt
rm -f entry_msg_types.txt

exit 0
