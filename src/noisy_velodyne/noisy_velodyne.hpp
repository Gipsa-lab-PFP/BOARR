#ifndef DEF_TEST
#define DEF_TEST

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

class NoisyVelodyne
{
    public:
        NoisyVelodyne(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~NoisyVelodyne();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber pointCloudSub_;
        ros::Publisher pointCLoudPub_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber imgSub_;
        image_transport::Publisher imgPub_;
        void imgCallback(const sensor_msgs::ImageConstPtr& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        double randNormal(double mean, double stddev);
};
#endif
