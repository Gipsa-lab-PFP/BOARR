#!/bin/bash

echo "Starting One Perfect Sensing Test"

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

echo "Starting the World and the RotorS simulator"  
timeout 130s roslaunch avoidance_benchmark perfect_benchmark_step1.launch world_name:=2D_100 & # > /dev/null 2>&1 & 
Proc1=$!
sleep 5s
if  (( $launchExecNumber > 0 )); then
    echo "Calling the launch files in order with 2s between each call"
    for((j=0; j<launchExecNumber; j++))
        do
            tmp=P$j
            timeout 125s roslaunch avoidance_benchmark $j.launch & # > /dev/null 2>&1 &
            eval $tmp=$!
        done
else
    echo "Starting the executables in order with 2s between each start"
    for((j=0; j<execNumber; j++))
    do
            tmp=P$j
            timeout 125s rosrun avoidance_benchmark Exec$j > /dev/null 2>&1 &
            eval $tmp=$!
        done
fi
sleep 2s
echo "launching the benchmark manager"
timeout 123s roslaunch avoidance_benchmark perfect_benchmark_step2.launch savingPrefix:=benchmark_results/$dirname/ showLiveImage:=true saveImage:=true saveVideo:=true & # > /dev/null 2>&1 & 
Proc2=$!
sleep 2s
echo "Waiting for the completion of the test" 
wait $Proc2
echo "the benchmark manager has terminated, closing everything that's still open"
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
echo "Everything have been killed"
echo "End of Avoidance Benchmark Script"
mkdir ../results/generated_files > /dev/null 2>&1 
mkdir ../results/generated_files/$dirname
cp ~/.ros/benchmark_results/$dirname/* ../results/generated_files/$dirname/
