#!/bin/bash

export DELTA_MSEC=20
export MAX_DELAY_MSEC=100
export GRPC_PORT=50051
export CORE_IPADDR=127.0.0.1
export PATH="/usr/local/bin/hakoniwa:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${DYLD_LIBRARY_PATH}"
ASSET_DEF="workspace/runtime/asset_def.txt"
#bash utils/rm_ipcs.bash

HAKO_CONDUCTOR_PID=
HAKO_ASSET_PROG_PID=
IS_OCCURED_SIGEVENT="FALSE"
function signal_handler()
{
    IS_OCCURED_SIGEVENT="TRUE"
    echo "trapped"
}

function kill_process()
{
    echo "trapped"
    if [ -z "$HAKO_CONDUCTOR_PID" ]
    then
        exit 0
    fi
    
    # HAKO_ASSET_PROG_PID に保存されている各PIDをkill
    for pid in $HAKO_ASSET_PROG_PID; do
        echo "KILLING: ASSET PROG $pid"
        kill -s TERM $pid || echo "Failed to kill ASSET PROG $pid"
    done
    
    echo "KILLING: hako-master-rust  $HAKO_CONDUCTOR_PID"
    kill -9 "$HAKO_CONDUCTOR_PID" || echo "Failed to kill hako-master-rust "

    while [ 1 ]
    do
        NUM=$(ps aux | grep hako-master-rust  | grep -v grep | wc -l)
        if [ $NUM -eq 0 ]
        then
            break
        fi
        sleep 1
    done

    exit 0
}

OLD_PID=`ps aux | grep hako-master-rust  | grep -v grep | awk '{print $2}'`
if [ -z $OLD_PID ] 
then
    :
else
    echo "KILLING old pid: ${OLD_PID}"
    kill -s TERM $OLD_PID
fi
trap signal_handler SIGINT

echo "INFO: ACTIVATING HAKO-CONDUCTOR"
hako-master-rust  ${DELTA_MSEC} ${MAX_DELAY_MSEC} ${CORE_IPADDR}:${GRPC_PORT} ${UDP_SRV_PORT} ${UDP_SND_PORT} ${MQTT_PORT} &
HAKO_CONDUCTOR_PID=$!

sleep 1

HAKO_ASSET_PROG_PID=
for entry in `cat ${ASSET_DEF}`
do
    TYPE=`echo $entry | awk -F: '{print $1}'`
    PROG=`echo $entry | awk -F: '{print $2}'`
    echo "${PROG}" | grep "^#"
    if [ $? -ne 0 ]
    then
        echo "INFO: ACTIVATING $entry"
        if [ ${TYPE} = "binary" ]
        then
            $PROG &
            HAKO_ASSET_PROG_PID="$! ${HAKO_ASSET_PROG_PID}"
        else
            echo "ERROR: NOT SUPPORTED TYPE=${TYPE}"
        fi
    fi
    sleep 1
done

echo "START"
while true; do
    echo "Press ENTER to stop..."
    read input
    if [ -z "$input" ]; then
        kill_process
        break
    fi
done

