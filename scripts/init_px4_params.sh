#!/bin/bash

env

# run simulation
timeout 1000s roslaunch avoidance_benchmark gui:=false interactive:=false benchmark_px4_step1.launch&
Proc1=$!

sleep 5s

# wait for simulation data
rostopic echo -n 1 /mavros/global_position/raw/fix

# sleep 10 seconds
sleep 10s

BOARR_DIR=`rospack find avoidance_benchmark`

echo "SET PX4 PARAMETERS"
rosrun mavros mavparam load $BOARR_DIR/param/px4_iris_params.txt

sleep 5s

kill $Proc1
wait $Proc1