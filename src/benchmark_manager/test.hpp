#ifndef DEF_TEST
#define DEF_TEST

#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/ModelStates.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ContactsState.h>
#include <topic_tools/MuxSelect.h>
#include <mavros_msgs/ActuatorControl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "util.hpp"


class Test
{
    public:
        Test();
        bool stop();
        void finalize();
        bool correctInitialization(){return _correctInitialization;}

    private:
        //trucs utilisés
        void initParam();
        
        ros::NodeHandle _nh;
        ros::Subscriber _worldSub;
        ros::Subscriber _odomSub;
        ros::Subscriber _collisionSub; 
        ros::Subscriber _motorSub;

        //used in core guiding functions
        ros::Publisher _trajectoryPub,_globalGoalPub, _globalGoalGpsPub;
        ros::ServiceClient _controlMuxClient;
        void initialHovering();
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
        void timerCallback(const ros::TimerEvent& timer); 
        void publish_x_y_z_yaw(double x, double y, double z, double new_yaw);

        ros::Timer _timer;    
        std::vector<Cart> _goals;
        std::vector<double> _goalsMinDistToValidate;
        std::vector<Odometry> _odomVector;
        bool _initialHoverTODO;
        Cart _initialHoverPos;
        double _initialHoverDuration, _initialHoverYaw;
        double _currentGoalNumber;
        double _testMaxTime, _finalLending, _stopOnCollision;
        double testTime;
        int _controlType;
        bool _stop, _correctInitialization;
        ros::Time testBeginTime, testEndTime;
        std::string _mavName;

        //used in code statiscal functions
        void collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
        void motorCallback(const mavros_msgs::ActuatorControlConstPtr& msg);
        int _collisionNumber;
        double linearDist;
        double consumedEnergy, prevP;
        ros::Time prevT;
        double travelledDistance;
        ros::Time initialCollisionTime;
        std::vector<Cart> _collisions;
        bool _saveSummary;
        std::ofstream _outputSummaryFile;
        std::string _summaryName;
        
        //used in graphial output
        image_transport::ImageTransport it;
        image_transport::Subscriber _imgSub;
        void imgCallback(const sensor_msgs::ImageConstPtr& msg);
        void gazeboWorldCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void createImg();
        void initialDrawing();
        void addDataToBaseImg();
        void drawTable(cv::Mat &img);
        void addFrameSpecificContent();
        void draw_cross(cv::Mat &img, cv::Point position, cv::Scalar Color, int length, int width, int orientation);
        void addTargetToCam();
        cv::Point worldToImage(double world_x, double world_y);

        cv::VideoWriter outputVideo;
        cv::Mat _currentImg;
        cv::Mat _currentDrawing;
        bool _saveVideo, _saveImage, _showImg;
        std::string _imageName, _videoName;
        double _videoFreq;
        cv::Mat _drawingBase;
        ros::Time lastDroneDrownStamp;
        ros::Time _camStamp;
        bool _perfectTest;
        std::vector<Cart> _modelPos;
        bool _worldCallback_CALLED;
};
#endif
