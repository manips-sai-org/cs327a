#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./hw3_p3_simviz &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID  
}

sleep 0.001 

# launch controller
./hw3_p3_controller &
CONTROLLER_PID=$!

sleep 1

# launch interfaces server
python3 interface/server.py p3_interfaces_plotter.html &
SERVER_PID=$!

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller & interfaces server
kill $CONTROLLER_PID
for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done