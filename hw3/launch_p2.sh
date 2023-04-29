#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./hw3_p2_simviz &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID  
}

sleep 0.001 

# launch controller
./hw3_p2_controller $1&
CONTROLLER_PID=$!

sleep 0.1

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller
kill $CONTROLLER_PID
