#include "noisy_realsense.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

NoisyRealsense::NoisyRealsense(
        const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    :nh_(nh),
    nh_private_(nh_private),
    it_(nh_)
{
    imgSub_ = it_.subscribe("raw_realsense", 1, &NoisyRealsense::imgCallback, this); 
    perfectImgPub_ = it_.advertise("perfect_realsense", 1);
    noisyImgPub_ = it_.advertise("noisy_realsense", 1);

    ros::Duration(.1).sleep(); // This sleep is also needed to let all the ros stuff initialize itself
}

NoisyRealsense::~NoisyRealsense() { }

void NoisyRealsense::imgCallback(const sensor_msgs::ImageConstPtr& msg)
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
    
    // mono16 depth image is expressed in mm
    if(msg->encoding==sensor_msgs::image_encodings::MONO16
       ||msg->encoding==sensor_msgs::image_encodings::TYPE_16UC1)
    {
	cv_ptr->image*=0.001;
    }
    
    cv::Mat inputImg;
    cv_ptr->image.copyTo(inputImg);

    // from a perfect camera, reduce the max depth to 20m
    // set the pixels of infinite depth (nan) at 20m
    double CameraRangeMax = 20;  
    cv::Mat nan_mask = cv::Mat(inputImg != inputImg);
    cv::Mat zero_mask = cv::Mat(inputImg == 0.);
    inputImg.setTo(cv::Scalar(CameraRangeMax), nan_mask | zero_mask);
    // threshold all pixels that are further than 20m 
    cv::threshold(inputImg, inputImg, CameraRangeMax,CameraRangeMax, CV_THRESH_TRUNC ); 
    
    // first noising step :
    // - reduce the size of the input image (not realistic -> removed)
    // - randomly move some pixels to the right (the horizontal baseline makes it harder to find the right horizontal position of the pixels)
    // - resize the image to the input size
//     cv::Mat small;
//     cv::resize(inputImg, small, cv::Size(), 1/4.0, 1/4.0, cv::INTER_LINEAR );
//     int x, y;
//     for ( int it = 0; it < 1000 ; it ++ )
//     {
//          int decal = 1 + (int)(rand()/(double)RAND_MAX * 4 );
//          x = 1 + (int)(rand()/(double)RAND_MAX * ( small.cols - decal - 1 ));
//          y = 1 + (int)(rand()/(double)RAND_MAX * ( small.rows - decal - 1 ));
//          small.at<float>(y,x) = small.at<float>(y,x - decal);
//     }
//     cv::Mat noised1;
//     cv::resize(small, noised1, inputImg.size(), 0,0, cv::INTER_LINEAR);

    cv::Mat noised1;
    inputImg.copyTo(noised1);
    int x, y;
    for ( int it = 0; it < 4000 ; it ++ )
    {
	int decal = 1 + (int)(8 * rand() / (double)RAND_MAX );
	x = 1 + (int)(rand()/(double)RAND_MAX * ( noised1.cols - decal - 1 ));
	y = 1 + (int)(rand()/(double)RAND_MAX * ( noised1.rows - decal - 1 ));
	noised1.at<float>(y,x) = noised1.at<float>(y,x - decal);
    }


    // on this modified image, find the edges (they will be set to nan as the last step)
    cv::Mat edges;
    int kernel_size = 7;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32F;
    cv::Sobel(noised1, edges, ddepth, 1, 0, kernel_size, 1, 0, cv::BORDER_DEFAULT );

    // on the modified image, add some local noise depending on the distance of each pixel and a random factor
    // to do so, create a gaussian matrix of medium size
    int kernelSize = 51; //size of the gaussian kernel
    cv::Mat linekernel, colkernel, gaussianKernel; 
    colkernel = cv::getGaussianKernel(kernelSize,kernelSize/3.0,CV_32F); 
    double min, max;
    minMaxIdx(colkernel, &min, &max);
    colkernel /= max;
    cv::transpose(colkernel, linekernel);
    gaussianKernel = colkernel * linekernel;

    // define the noise level
    double maxNoisePower = 0.02; //multiplied by the squared depth it gives a 2m maximum noise at 10m, 50cm at 5m, and 2.5cm at 1m

    // insert a number of those gaussian matrix with a random factor and depth factor in the image
    for ( int gaussianNoiseNumber = 0; gaussianNoiseNumber < 100 ; gaussianNoiseNumber ++ )
    {
         x = (int)(rand()/(double)RAND_MAX * ( noised1.cols - kernelSize ));
         y = (int)(rand()/(double)RAND_MAX * ( noised1.rows - kernelSize ));
         double noisePower = 2.0* (rand()/(double)RAND_MAX * maxNoisePower) - maxNoisePower; //symetric noise (objects can be seen further of closer to the camera)
         cv::Mat noiseMatrix, noiseMatrixTrunc;
         noiseMatrix = noised1(cv::Rect(x,y,kernelSize,kernelSize)).mul(noised1(cv::Rect(x,y,kernelSize,kernelSize))).mul(gaussianKernel*noisePower); //noise = rand*d²*gaussianDistrib
         cv::threshold(noiseMatrix, noiseMatrixTrunc, 2.0, 2.0, cv::THRESH_TRUNC);
         noised1(cv::Rect(x,y,kernelSize,kernelSize)) = noised1(cv::Rect(x,y,kernelSize,kernelSize)) + noiseMatrixTrunc;
    }
    // End of the noise add, send the image back
    
    // add a slight gaussian noise that seem existant in the realsense and that will lower the impact of the double resizing.
    cv::Mat outImg; 
    //cv::GaussianBlur( noised1, outImg, cv::Size(7,7), 7.0/3.0, 7.0/3.0, cv::BORDER_DEFAULT); 
    cv::bilateralFilter(noised1, outImg, 7, 10, 10, cv::BORDER_DEFAULT);
    
    // set to nan all horizontal edges that are greater than a treshold
    cv::Mat out_nan_mask = cv::Mat(edges > 2048.0/4.0); //2048 -> 1 normalized meter according to the opencv doc ( normFactor : 2**(k_size*2-dx-dy-2)
    outImg.setTo( 0. /*std::nan("1")*/, out_nan_mask);

    // last step, both for the perfect and the noisy images, set everything that's under 53cm (see Intel R©RealSenseTM Stereoscopic Depth Cameras, arxiv2017) to nan
    cv::Mat inf53cm = cv::Mat(inputImg < 0.53 );
    outImg.setTo( 0./*std::nan("1")*/, inf53cm );
    inputImg.setTo( 0. /*std::nan("1")*/, inf53cm );

    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, outImg).toImageMsg();
    this->noisyImgPub_.publish(imgMsg);

    sensor_msgs::ImagePtr imgMsg2 = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, inputImg).toImageMsg();
    this->perfectImgPub_.publish(imgMsg2);

}

double NoisyRealsense::randNormal(double mean, double stddev)//Box muller method
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
