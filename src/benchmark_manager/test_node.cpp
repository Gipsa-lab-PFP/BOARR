#include <ros/ros.h>
#include "test.hpp"

int main(int argc, char** argv)
{
  //initialization
  ros::init(argc, argv, "benchmark_manager_node");
  Test test;
  
  //running the test
  while ( !test.stop() )
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(10));;
  }
 
  // checking if we exit because of an initialiation issue and return an error code
  // more importantly, we exit before finalize() which writes in the output file
  // this will result in relaunching the exact same test when doing the full benchmark
  if ( !test.correctInitialization() ) { return 1; } 

  //creating the outputs
  test.finalize();

  return 0;
}
