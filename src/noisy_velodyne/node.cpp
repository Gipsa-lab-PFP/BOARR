#include <ros/ros.h>
#include "noisy_velodyne.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "noisy_velodyne_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    NoisyVelodyne noisyVelodyne(nh, private_nh);

    ros::spin();
    return 0;
}
