#!/bin/bash

echo "Starting the Perfect Sensing Exemple Script"  

# Creating the directory that will contain the output of the test
mkdir ~/.ros/benchmark_results > /dev/null 2>&1 
dirname=`date +%Y_%m_%d-%Hh%Mm%Ss`
mkdir ~/.ros/benchmark_results/$dirname 

echo "Starting the World and the RotorS simulator (No GUI by default)"  
timeout 120s roslaunch avoidance_benchmark perfect_benchmark_step1.launch world_name:=2D_50 > /dev/null 2>&1 & 
P1=$!
sleep 5s
echo "Starting the Example Avoidance Algorithm"
timeout 115s roslaunch avoidance_benchmark dummy_avoidance_algorithm.launch  > /dev/null 2>&1 & 
P2=$!
sleep 2s
echo "Starting the Benchmark Manager"
timeout 113s roslaunch avoidance_benchmark perfect_benchmark_step2.launch savingPrefix:=benchmark_results/$dirname/ showLiveImage:=true saveImage:=true saveVideo:=true & # > /dev/null 2>&1 & 
P3=$!
sleep 2s
echo "Waiting for the completion of the test" 
wait $P3
kill $P2
wait $P2
kill $P1
wait $P1
echo "Everything have been killed, end of the bash script"
mkdir ../results/generated_files > /dev/null 2>&1 
mkdir ../results/generated_files/$dirname
cp ~/.ros/benchmark_results/$dirname/* ../results/generated_files/$dirname/
