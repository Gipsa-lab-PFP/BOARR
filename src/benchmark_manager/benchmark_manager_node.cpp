#include <ros/ros.h>
#include "BenchmarkManager.hpp"

int main(int argc, char** argv)
{
  //initialization
  ros::init(argc, argv, "benchmark_manager_node");
  BenchmarkManager benchmarkManager;
  
  //running the test
  ros::Rate rate(100.0);
  while ( !benchmarkManager.stop() )
  {
      ros::spinOnce();
      rate.sleep();
      //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(10));;
  }
  
  ros::spinOnce();
  
  // checking if we exit because of an initialiation issue and return an error code
  // more importantly, we exit before finalize() which writes in the output file
  // this will result in relaunching the exact same test when doing the full benchmark
  if ( !benchmarkManager.correctInitialization() ) { return 1; } 

  //creating the outputs
  benchmarkManager.finalize();

  return 0;
}
