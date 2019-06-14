#!/bin/bash

echo "Starting One Noisy Test"

worldNumber=$(find ../gazebo/worlds/ -type f | grep ".world$" | wc -l)
echo "Number of Available worlds : $worldNumber"

if (( $worldNumber == 0 )); then
    echo "No World Found, Exiting"
    exit 1
fi

execNumber=$(find ../testedAlgorithm/ -type f | grep "Exec" | wc -l)
echo "Number of Executables to start: $execNumber"

launchExecNumber=$(find ../testedAlgorithm/ -type f |grep ".launch$" | wc -l)
echo "Number of launch to call: $launchExecNumber"

if (( $launchExecNumber > 0 )); then
    echo "Launch(s) Found, The Tested Avoidance Algorithm will be started through the provided launch files"
elif (( $execNumber == 0 )); then
    echo "No Executable Nor launch found with the appropriate names in the testedAlgorithm directory, Exiting"
    exit 1
else
    echo "No Launch Found, Starting the executables directly"
fi

# Creating the directory that will contain the output of the test
mkdir ~/.ros/benchmark_results > /dev/null 2>&1 
dirname=`date +%Y_%m_%d-%Hh%Mm%Ss`
mkdir ~/.ros/benchmark_results/$dirname 

# Setting spawn coordinates as environment variables
read -a txtArray < ../gazebo/worlds/forest2.yaml
spawnX=${txtArray[2]}
spawnY=${txtArray[5]}
spawnZ=${txtArray[8]}
export spawnX
export spawnY
export spawnZ

echo "Starting the World and the RotorS simulator (No GUI by default)"  
timeout 1000s roslaunch avoidance_benchmark benchmark_step1.launch world_name:=worlds/forest2 & # > /dev/null 2>&1 & 
Proc1=$!
sleep 5s
if  (( $launchExecNumber > 0 )); then
    echo "Calling the launch files in order with 2s between each call"
    for((j=0; j<launchExecNumber; j++))
        do
            tmp=P$j
            timeout 1000s roslaunch avoidance_benchmark $j.launch & # > /dev/null 2>&1 &
            eval $tmp=$!
        done
else
    echo "Starting the executables in order with 2s between each start"
    for((j=0; j<execNumber; j++))
    do
            tmp=P$j
            timeout 1000s rosrun avoidance_benchmark Exec$j > /dev/null 2>&1 &
            eval $tmp=$!
        done
fi
sleep 2s
echo "launching the benchmark manager"
timeout 1000s roslaunch avoidance_benchmark benchmark_step2.launch world_name:=worlds/forest2 savingPrefix:=benchmark_results/$dirname/ showLiveImage:=true saveImage:=true saveVideo:=true &
Proc2=$!
sleep 2s
echo "Waiting for the completion of the test" 
wait $Proc2
if  (( $launchExecNumber > 0 )); then
    for((k=launchExecNumber-1;k>=0;k--))
        do
            eval tmp='$'P$k
            kill $tmp
            wait $tmp
        done
else
    for((k=execNumber-1;k>=0;k--))
        do
            eval tmp='$'P$k
            kill $tmp
            wait $tmp
        done
fi
kill $Proc1
wait $Proc1
echo "Everything have been killed, End of the script"
mkdir ../results/generated_files > /dev/null 2>&1 
mkdir ../results/generated_files/$dirname
cp ~/.ros/benchmark_results/$dirname/* ../results/generated_files/$dirname/
