#!/bin/bash

if [[ $(uname -a | grep -q microsoft) -eq 0 ]]
then
    OS_TYPE=wsl2
elif [[ $(uname -a | grep -q Dawrin) -eq 0 ]]
then
    OS_TYPE=Mac
else
    OS_TYPE=Linux
fi

echo ${OS_TYPE}