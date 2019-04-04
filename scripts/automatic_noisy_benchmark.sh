#!/bin/bash

echo "Starting the Noisy Benchmark Script"

worldNumber=$(find ../gazebo/benchmark_v1.0_worlds/ -type f | grep ".world$" | wc -l)
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
    echo -e "Launch(s) Found, The Tested Avoidance Algorithm will be started through the provided launch files \n"
elif (( $execNumber == 0 )); then
    echo -e "No Executable Nor launch found with the appropriate names in the tested Algorithm directory, Exiting \n"
    exit 1
else
    echo -e "No Launch Found, Starting the executables directly \n"
fi

# Creating the directory that will contain the output of the test
mkdir ~/.ros/benchmark_results > /dev/null 2>&1 
dirname=`date +%Y_%m_%d-%Hh%Mm%Ss`
mkdir ~/.ros/benchmark_results/$dirname 

for ((i=0; i<worldNumber; i++))
    do
        # Setting spawn coordinates as environment variables
        read -a txtArray < ../gazebo/benchmark_v1.0_worlds/forest$i.yaml
        spawnX=${txtArray[2]}
        spawnY=${txtArray[5]}
        spawnZ=${txtArray[8]}
        export spawnX
        export spawnY
        export spawnZ
        
        echo -e "Starting the World and the RotorS simulator for the $((i+1)) time"  
        timeout 175s roslaunch avoidance_benchmark benchmark_step1.launch world_name:=benchmark_v1.0_worlds/forest$i & #> /dev/null 2>&1 & 
        Proc1=$!
        sleep 5s
        if  (( $launchExecNumber > 0 )); then
            echo "Calling the launch files in order with 2s between each call"
            for((j=0; j<launchExecNumber; j++))
                do
                    tmp=P$j
                    timeout 170s roslaunch avoidance_benchmark $j.launch > /dev/null 2>&1 &
                    eval $tmp=$!
                    sleep 2s
                done
        else
            echo "Starting the executables in order with 2s between each start"
            for((j=0; j<execNumber; j++))
            do
                    tmp=P$j
                    timeout 170s rosrun avoidance_benchmark Exec$j > /dev/null 2>&1 &
                    eval $tmp=$!
                    sleep 2s
                done
        fi
        echo "Launching the benchmark manager"
        timeout 30s roslaunch avoidance_benchmark benchmark_step2.launch world_name:=benchmark_v1.0_worlds/forest$i savingPrefix:=benchmark_results/$dirname/ & # > /dev/null 2>&1 &
        #timeout 168s roslaunch avoidance_benchmark benchmark_step2.launch world_name:=benchmark_v1.0_worlds/forest$i savingPrefix:=benchmark_results/$dirname/ > /dev/null 2>&1 &
        Proc2=$!
        sleep 2s
        echo "Waiting for the completion of the test" 
        wait $Proc2
        echo "The benchmark manager has been closed, closing everything that's still open"
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
        sleep 15s
    done
echo "End of the Tests, RESULTS : "
cp ~/.ros/benchmark_results/$dirname/default_summary ../results/$dirname
python3 statisticalAnalysis.py ../results/$dirname
echo "End of Avoidance Benchmark Script"
