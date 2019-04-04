## MAJOR :
Doc :
- Write a doc with statistical explanations for the number of tries of the benchmark
- Write the ComputeTheAvoidanceIndicators doc file
- Add the name of the topics to the UseYourOwnAlgorithm doc file
Features :
- redefine the sensors noise according to the sensor described in the the paper !!
- link WIND PLUGIN in a general way !!!!!!!!!!!!!!!!!!!!!!
- wind plugin (called in xacro files) => add argument to it; call it with a "wind_profile" that gives the wind and gust power and a "wind direction" and "gust direction" in the launch file
- wind plugin conversion from wind vel to Forces using augustin manecy thesis and/or "SIMULATION OF WIND EFFECT ON A QUADROTOR FLIGHT" 2015. 

## Minor :
Durability :
- Edit the README of my RotorS fork to link to my fork instead of the ethz repo ( in order to have a real backup solution if some modifications of the ETHZ repo break the compability )
- Fork all the packages from the RotorS rosintall, especially mav_comm which is a RotorS dependancy.
Features :
- an img noised velodyne exist but there should also be a point cloud noised velodyne
- add a matplotlib based python script to plot the histograms of the indicators. 

## Possible ( but desirable ? ) :
- put the benchmark result file directly into the avoidance bencmark/results directory instead that in .ros
            (easier access but the video files are big and the .ros size is checked and a message is send to ask to clean it ).
- add a STOP AT FIRST COLLISION for the statistical benchmark ( should be done as a benchmark_manager parameter )
