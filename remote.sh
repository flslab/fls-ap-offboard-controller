#!/bin/bash

REMOTE_IP="192.168.1.73"
REMOTE_USER="fls"
REMOTE_DIR="/home/fls/fls-ap-offboard-controller"
VENV_PATH="/home/fls/env"
PYTHON_SCRIPT="controller.py"
LOG_DIR="./logs"

PYTHON_ARGS="$@"

ssh ${REMOTE_USER}@${REMOTE_IP} \
    "cd $REMOTE_DIR && \
     source $VENV_PATH/bin/activate && \
     python $PYTHON_SCRIPT $PYTHON_ARGS" 2>&1 | tee remote_output.log

LOG_PATH=$(grep 'Vicon log saved in' remote_output.log | sed -n 's/.*logs\/\(vicon[^ ]*\.json\).*/logs\/\1/p')

if [ -z "$LOG_PATH" ]; then
    echo "Could not find log file path in output."
    exit 1
fi

scp ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DIR}/${LOG_PATH} ${LOG_DIR}

echo "Log file downloaded: $(basename $LOG_PATH)"
