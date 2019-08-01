# The Quadrotor Avoidance Benchmark

This package main purpose is to test and compare the multiple existing avoidance algorithm for quadrotors with the hope that comparing the results could help choosing promising directions for future developments on autonomous obstacle avoidance.

## How does it work ?
The avoidance benchmark consist mainly of a quadrotor simulator, multiple custom created worlds, a benchmark manager and some scripts to compute the benchmark indicators. It is based on Ros Kinetic & Gazebo 7.0.0.
The simulator is RotorS from the ETHZ which is both the most common multirotor model in ROS and the closest simulator to a real multirotor. On the 01/04/2019, it is possible to use the main branch from the ETHZ project available [here](https://github.com/ethz-asl/rotors_simulator). It will probably be possible to use this main branch in the future. In case there are some major modifications that break the compability between their project and mine, please create an issue and I will fork RotorS.
Worlds are created through a custom world plugin that has been created specifically for this purpose. This plugin generates GAZEBO forests of 6700m2. The density of the forest is a parameter during its creation which allow some diversity in the test environments. The chosen environment is a forest since it is in the literature the most common environment to test avoidance algorithms for quadrotors.
The benchmark manager send the coordinates where the drone should go and log informations in order to output logical indicators to characterize the tests.
Finally, the scripts allow the use of the package in a few simple operations for simplicity.

## Documentation

This ReadMe will only focus on how to install and use the demonstrator of the package.

Most of the documentation is available in the 'doc' directory. There you can find informations on :
- how to use your own avoidance algorithm during the tests
- how to compute the avoidance indicators for your own algorithm
- how to use the world creation plugin
- the current TODO list and some other minor doc files.

## Citations
Feel free to use this package at your convenience but please cite the following paper if you do so.
```bibtex
@article{du2019boarr,
  title={BOARR: A Benchmark for quadrotor Obstacle Avoidance based on ROS and RotorS},
  author={Du Montcel, Thibaut Tezenas and Negre, Amaury and Gomez-Balderas, Jose-Ernesto and Marchand, Nicolas},
  year={2019}
}
```

 If your usage include the RotorS simulator, please cite the ETHZ for their simulator :
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```

## Install the benchmark and use the demonstrator


#### If you already are a ROS user AND have the RotorS simulator :
1. Clone the repo into the src folder of a standard ros workspace
```
$ cd (your_catkin_ws_path)/src/
$ git clone git@github.com:ttdm/rotors_simulator.git
```
2. Compile your workspace using catkin_make
3. Add a path to your GAZEBO_MODEL_PATH and you GAZEBO_PLUGIN_PATH :
```
$ export GAZEBO_PLUGIN_PATH=(Your avoidance_benchmark path)/gazebo/worlds_creation_plugin/build:$GAZEBO_PLUGIN_PATH
$ export GAZEBO_MODEL_PATH=(Your avoidance_benchmark path)/gazebo/models:$GAZEBO_MODEL_PATH
```
For convenience, you should add it to your ~/.bashrc

4. Start one of the two example algorithm :
```
$ cd (your avoidance_benchmark path)/scripts
```
then
```
$ ./dummy_noisy_benchmark
```
or
```
$ ./dummy_perfect_benchmark
```

#### If you don't have the RotorS simulator package but if you are not new ros user

1. Install rotors and its dependencies following their instructions https://github.com/ttdm/rotors_simulator. Please be careful to create a new specific workspace since they chose to use the new "catkin build" from catkin_tools over the more classical catkin_make that is commonly used with ROS.

2. source it and add the source in your bash rc:
```
$ source ~/catkin_ws/devel/setup.bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
3. Go to the previous chapter to install and use the avoidance benchmark.


#### If you are a new ROS user :
1. Install ROS, RotorS and all their dependencies following the ETHZ instructions https://github.com/ttdm/rotors_simulator.

2. Since RotorS uses a new build tool among the catkin_tools over the more classical one that is used by most ROS package, we will need to create a new workspace in which we will use the older build tool :
```
$ mkdir -p ~/catkin_make_ws/src
```
3. Clone the package into the workspace
```
$ cd ~/catkin_make_ws/src/
$ git clone git@github.com:ttdm/rotors_simulator.git
```

4. Compile the package using catkin_make :
```
$ cd ..
$ catkin_make
```

5. Start one of the two example algorithm :
```
$ cd ~/catkin_make_ws/src/avoidance_benchmark/scripts
```
then
```
$ ./dummy_noisy_benchmark
```
or
```
$ ./dummy_perfect_benchmark
```

## Outputs

In your ~/.ros directory, a new benchmark_results directory has been created. Each time you use a script, a directory with the date and hour of launch will be created.
Inside, you will always find a default_summary file. This file saves the value of the computed indicators of each flight. When doing hundreds of simulated flights, it is then the parsed file to compute the global indicators.
Depending on the script you used, a default_image.jpg and a default_video.avi will also be created. They allow you to witness precisely what happen during your attempt. Those files are created during the demonstrator script.
Using the scripts will also create a subdirectory in the result directory in which you will find all the automatically generated files. 
