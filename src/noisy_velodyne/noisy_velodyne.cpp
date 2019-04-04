#include "noisy_velodyne.hpp"

#include <cv_bridge/cv_bridge.h>

NoisyVelodyne::NoisyVelodyne(
        const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    :nh_(nh),
    nh_private_(nh_private),
    it_(nh_)
{
    imgSub_ = it_.subscribe("perfect_img_velodyne", 1, &NoisyVelodyne::imgCallback, this); 
    pointCloudSub_ = nh_.subscribe("perfect_cloud_velodyne", 1, &NoisyVelodyne::pointCloudCallback, this); 
    imgPub_ = it_.advertise("noised_img_velodyne", 1);
    pointCLoudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("noised_cloud_velodyne", 1);

    ros::Duration(.1).sleep(); // This sleep is also needed to let all the ros stuff initialize itself
}

NoisyVelodyne::~NoisyVelodyne() { }

void NoisyVelodyne::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        std::cout << "Benchmark Manager: Error during the depth callback" << std::endl; 
        return;
    }
    cv::Mat inputImg;
    cv_ptr->image.copyTo(inputImg);

    cv::Mat mGaussian_noise = cv::Mat(inputImg.size(),CV_32F);
    cv::randn( mGaussian_noise, 0, 0.01 );

    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, inputImg+mGaussian_noise).toImageMsg();
    this->imgPub_.publish(imgMsg);
}

// TODO below is work in progress
#include <iostream>

void NoisyVelodyne::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //TODO working directly on data doesnt seem to work; the data size is also weird compared to what's ; pcl use seems mandatory; TODO
    sensor_msgs::PointCloud2 new_msg;
    new_msg = *msg; 
    
    //static_assert(sizeof(float) == 4);
    //float f = 0; // whatever value
    //uint8_t bytes[4];
    //std::memcpy(bytes, &f);
    //for (int i = 0; i < new_msg.row_step * new_msg.height; i+=4)
    //{
        //float val = *(float*)(&new_msg.data[i]);
            //std::cout << "val " << val << std::endl; 
        //std::getchar();
        //new_msg.data[i] = new_msg.data[i] ;// + randNormal(0, 2);//0.01);
    //}
    this->pointCLoudPub_.publish(new_msg);
    //height ; 480  width 640  bigendian false
    //point step : 32  row step 20480  isdense false
    //FIELD 0: name x  offset 0  datatype  7 count 1
    //FIELD 1: name y  offset 4  datatype  7 count 1
    //FIELD 2: name z  offset 8  datatype  7 count 1
    //FIELD 3: name rgb  offset 16  datatype 7  count 1
    // all datatype = float32
    // 20480 / 32 = 640
    // data possède 9 963 648 éléments (iterating=>
    // row step * height 9 830 400 => ... doesnt fit the doc ... 
    // row step = 32 * row pix => 4*8 uint8 => 4 float /pix
    //std::cout << " and a gethar to rule them all " << std::endl;
    //std::getchar();
}

double NoisyVelodyne::randNormal(double mean, double stddev)//Box muller method
{
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/(double)RAND_MAX - 1;
            y = 2.0*rand()/(double)RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}
