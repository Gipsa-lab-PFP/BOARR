## How to make my algorithm compatible with the benchmark ?

It's actually quite easy to have a compatible algorithm.
The benchmark is designed to provide a set of ROS topic as inputs and outputs.

The entry topics are :
- an odometry topic
- an imu topic
- a gps topic
- a depth camera topic
- a single centered looking ahead camera
- a point cloud
- a looking ahead camera
- a looking downward camera

For the output topic, you have the choice between :
- directly sending motor commands
- sending a RPYrT command. The angular control will then be completed using a custom, available here, attitube controller which gives better flight results than the one available in the original rotoRs package.

The best way to check for compatibility is to clone rotors and this repo localy and to try and launch your algorithm on your local copy of the benchmark.
RotoRs is a catkin workspace by itself, it should be builded using the catkin build command and it path should then be added to your /.bashrc. For more details, please see here.
This repo is a classic package, it should be added to an already existing catkin_workspace and build with a simple catkin_make call.
There are 2 possible degree of compatibility 1. working in the environment algorithm or 2. ready to be submitted algorithm

To setup
1. To test if your algorithm is working with the benchmark, the easiest way is to :
- launch the working_test1.launch
- launch or rosrun all your algorithm components
- launch the working_test_manager.launch

If you correctly remapped all your inputs/outputs, you should see a quadrotor, by default a hummingbird, attempting to complete the avoidance test on a map.

2. To test if you algorithm is ready to be submitted, you have to find all the executables that you usually start (all the nodes). Copy them into the exec_to_test directory while renaming them Exec0->99.
Please note that your Executable should almost be self containing, they shouldn't use other external libraries than openCV and catkin libraries.
    Please also note that custom messages and services are currently not supported, if you used some custom messages, you should split them or regroup them into "classical" message that are defined in official open source ros packages.
    Copy your launch files if you used some, in the same exec_to_test algorithm and renaming them 0-99.launch, modifying the name of the nodes to correspond to their change of name and location.
    To launch the avoidance algorithm on the 10 test worlds available with the package, you then simply need to start the automatic_benchmark.sh shell script.
