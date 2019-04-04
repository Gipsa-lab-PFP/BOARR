#include "dummyAvoidance.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_avoidance");
  ros::NodeHandle nh;
  
  DummyAvoidance dummyAvoidance(nh);
  
  ros::spin();
  return 0;
}
