#!/bin/bash

echo "Starting the Perfect Sensing Benchmark"

execNumber=$(find ../testedAlgorithm/ -type f | grep "Exec" | wc -l)
echo "Number of Executables to start: $execNumber"

launchExecNumber=$(find ../testedAlgorithm/ -type f |grep ".launch$" | wc -l)
echo "Number of launch to call: $launchExecNumber"

if (( $launchExecNumber > 0 )); then
    echo -e "Launch(s) Found, The Tested Avoidance Algorithm will be started through the provided launch files \n"
elif (( $execNumber == 0 )); then 
    echo -e "No Executable Nor launch found with the appropriate names in the testedAlgorithm directory, Exiting \n"
    exit 1
else
    echo -e "No Launch Found, Starting the executables directly \n"
fi

# Creating the directory that will contain the output of the test
mkdir ~/.ros/benchmark_results > /dev/null 2>&1 
dirname=`date +%Y_%m_%d-%Hh%Mm%Ss`
mkdir ~/.ros/benchmark_results/$dirname 

for ((i=1; i<=1060; i++))
    do
        echo "Starting the World and the RotorS simulator for the $i time"  
        timeout 150s roslaunch avoidance_benchmark perfect_benchmark_step1.launch world_name:=2D_100 > /dev/null 2>&1 & 
        Proc1=$!
        sleep 5s
        if  (( $launchExecNumber > 0 )); then
            echo "Calling the launch files in order with 2s between each call"
            for((j=0; j<launchExecNumber; j++))
                do
                    tmp=P$j
                    timeout 145s roslaunch avoidance_benchmark $j.launch > /dev/null 2>&1 &
                    eval $tmp=$!
                done
        else
            echo "Starting the executables in order with 2s between each start"
            for((j=0; j<execNumber; j++))
            do
                    tmp=P$j
                    timeout 145s rosrun avoidance_benchmark Exec$j > /dev/null 2>&1 &
                    eval $tmp=$!
                done
        fi
        sleep 4s
        echo "launching the benchmark manager"
        timeout 140s roslaunch avoidance_benchmark perfect_benchmark_step2.launch stopOnCollision:=true savingPrefix:=benchmark_results/$dirname/ showLiveImage:=true > /dev/null 2>&1 & 
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
        echo "Everything has been killed."
        # checking that the test manager has correctly output something in the text file. 
        linenumber=$(cat ~/.ros/benchmark_results/$dirname/default_summary | wc -l)
        # if it did
        echo $linenumber 
        echo $((i+1))
        if (( $linenumber == $((i+1)) )); then
            echo -e "Waiting 5s before starting the next loop \n" 
            # if it did not: 
        else 
            i=$((i-1))
            echo -e "An ERROR ocurred, waiting 5s before restarting the SAME loop \n" 
        fi
        sleep 10s
    done
    
echo "End of the Tests, RESULTS : "
mkdir ../results/generated_files > /dev/null 2>&1 
mkdir ../results/generated_files/$dirname
cp ~/.ros/benchmark_results/$dirname/default_summary ../results/generated_files/$dirname
python3 statisticalAnalysis.py ../results/$dirname/default_summary
echo "End of Avoidance Benchmark Script"
