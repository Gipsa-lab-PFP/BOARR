#include <ros/ros.h>
#include "noisy_realsense.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "noisy_realsense_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    NoisyRealsense noisyRealsense(nh, private_nh);

    ros::spin();
    return 0;
}
