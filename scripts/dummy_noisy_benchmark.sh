#!/bin/bash

echo "Starting the Exemple Noisy Script"

worldNumber=$(find ../gazebo/worlds/ -type f | grep ".world$" | wc -l)
echo "Number of Available worlds : $worldNumber"

if (( $worldNumber == 0 )); then
    echo "No World Found, Exiting"
    exit 1
fi

# Creating the directory that will contain the output of the test
mkdir ~/.ros/benchmark_results > /dev/null 2>&1 
dirname=`date +%Y_%m_%d-%Hh%Mm%Ss`
mkdir ~/.ros/benchmark_results/$dirname 

# Setting spawn coordinates as environment variables
read -a txtArray < ../gazebo/worlds/forest0.yaml
spawnX=${txtArray[2]}
spawnY=${txtArray[5]}
spawnZ=${txtArray[8]}
export spawnX
export spawnY
export spawnZ

echo "Starting the World and the RotorS simulator (No GUI by default)"  
timeout 180s roslaunch avoidance_benchmark benchmark_step1.launch world_name:=worlds/forest0 & 
Proc1=$!
sleep 5s
echo "Starting the Example Avoidance Algorithm"
timeout 175s roslaunch avoidance_benchmark dummy_avoidance_algorithm.launch > /dev/null 2>&1 &
P1=$!
sleep 2s
echo "Starting the Benchmark Manager"
timeout 173s roslaunch avoidance_benchmark benchmark_step2.launch world_name:=worlds/forest0 savingPrefix:=benchmark_results/$dirname/ showLiveImage:=true saveImage:=true saveVideo:=true & #> /dev/null 2>&1 & 
Proc2=$!
sleep 2s
echo "Waiting for the completion of the test" 
wait $Proc2
kill $P1
wait $P1
kill $Proc1
wait $Proc1
echo "Everything has been killed, End of the Noisy Exemple Script" 
mkdir ../results/generated_files > /dev/null 2>&1
mkdir ../results/generated_files/$dirname
cp ~/.ros/benchmark_results/$dirname/* ../results/generated_files/$dirname/
python3 ../results/statisticalAnalysis.py ../results/generated_files/$dirname/default_summary
echo "End of the dummy exemple"
