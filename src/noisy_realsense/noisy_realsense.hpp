#ifndef DEF_TEST
#define DEF_TEST

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

class NoisyRealsense
{
    public:
        NoisyRealsense(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~NoisyRealsense();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber imgSub_;
        image_transport::Publisher perfectImgPub_, noisyImgPub_;
        void imgCallback(const sensor_msgs::ImageConstPtr& msg);
        double randNormal(double mean, double stddev);
};
#endif
