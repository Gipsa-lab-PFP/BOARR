## How to use the benchmark to compute the Avoidance Indicators ?

1. Verify that your algorithm is compatible with the benchmark (see **UseYourOwnAlgorithm.md**)
2. Extract the ``benchmark_v1.0_worlds.zip`` that you can find in gazebo/ and replace the ``benchmark_v1.0_worlds`` directory by the extracted directory (only 10 worlds are currently in the ``benchmark_v1.0_worlds`` directory to keep the github repo light while still offering the possibility to try the created scripts)
3. Open a terminal in the script directory
4. Execute the ``./automatic_noisy_benchmark`` script. 

	It will automatically do the 1060 tests needed to compute the collision probability with a +-0.05 precision and 99% chance to be in the confidence interval (see the paper 'BOARR : A Benchmark for quadrotor Obstacle Avoidance Based on ROS and RotorS' for the explanations on the number of tests)
	If you have python3, it will also print in the terminal the primary indicator with it's confidence interval and the secondary indicators and their first and last decile. 

If you don't have python3, you can go to your result directory, you will find the result as a text file called "FullBenchmarkResults.txt".
You can compute the indicators either :
- with python, using the "statisticalAnalysis.py" file that is compatible with python 2 and 3 and does not use any non standard package. 
	Since we choose to use only the core of python, this script cannot display any more information (such as the visuals of the full distribution of your results).
- with matlab using the "statisticalAnalysis.m" file, it is possible to display the indicators plus the distributions of the indicators.
