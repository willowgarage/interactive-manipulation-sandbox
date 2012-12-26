#! /bin/bash

###
# Allow dry-runs.
# Set to 'false' if you want the script to take any effect. Otherwise it will
# only echo the lines it would otherwise execute
#
DEBUG=false

runthis () {
    command_message=$1
    command_line=$2
    echo
    echo "--- ${command_message}... ---"
    if ${DEBUG} ; then
        echo ${command_line}
    else
        eval ${command_line}
    fi
    echo "--- ...done ---"
}

###
# Script variables.
#
# Auto-discover location of deploy script to calculate location of other files from there
DEPLOYDIR="`dirname \"$0\"`"
DEPLOYDIR="`( cd \"$DEPLOYDIR\" && pwd )`"
LAYER2DIR="${DEPLOYDIR}/../server"
PYTHON="python"
SERVER_PID_FILE="${LAYER2DIR}/server.pid"
#
###


###
# Instance variables.
CURRENT_SERVER_PID="$(cat ${SERVER_PID_FILE})"
DUMP_DIR="${LAYER2DIR}/dumps"
DUMP_FILE="layer2.$(date +'%Y%m%d').json"

###
# Tasks
#

run_server () {
    ${PYTHON} manage.py runserver &>/dev/null &
    CURRENT_SERVER_PID=$!
    echo ${CURRENT_SERVER_PID} > ${SERVER_PID_FILE}
}


###
# Move to the LAYER2DIR
runthis "cd'ing into the target directory" "cd ${LAYER2DIR}"

###
# Step 1:
#
# Kill the running server.
runthis "Killing the server" "kill ${CURRENT_SERVER_PID}"
# TODO: Wait for server to die and make sure it's dead before continuing
sleep 3

###
# Step 2:
#
# Run all migrations.
runthis "Running migrations" "${PYTHON} manage.py migrate --all"

###
# Step 3:
#
# Dump the "world" data to a known location for others to load.
runthis "Dumping today's world' data." "mkdir -p ${DUMP_DIR}; ${PYTHON} manage.py dumpdata --all | tail -n 1 > ${DUMP_DIR}/${DUMP_FILE}"

###
# Step 4:
#
# Bring the server back up.
runthis "Get the server back online" "run_server"
