#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation and control first
./hw5_p2&
HW5P2_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $HW5P2_PID
}

sleep 0.1 

# launch interfaces server
python3 interface/server.py hw5_interfaces_plotter.html &
SERVER_PID=$!

# wait for simviz to quit
wait $HW5P2_PID

# onnce simviz dies, kill interfaces server
for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
