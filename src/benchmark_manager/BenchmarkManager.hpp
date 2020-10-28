#ifndef __BENCHMARK_MANAGER_HPP__
#define __BENCHMARK_MANAGER_HPP__

#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "util.hpp"

class BenchmarkManager
{
public:
    BenchmarkManager();
    bool stop();
    void finalize();
    bool correctInitialization(){return _correctInitialization;}
    
private:
    enum ManagerState
    {
	SET_MUX,
	WAITING_CONNECTION,
	PRE_HOVERING,
	HOVERING,
	HOLDING,
	TESTING,
	FAILED,
	SUCCESSED
    };
    
    ManagerState _managerState;
    
    void initParam();
    
    ros::NodeHandle _nh;
    ros::Subscriber _worldSub;
    ros::Subscriber _odomSub;
    ros::Subscriber _collisionSub; 
    ros::Subscriber _linkStatesSub;
    ros::Subscriber _stateSub;
    
    std::vector<int> _rotor_links;
    

    //used in core guiding functions
    ros::Publisher _globalGoalPub, _globalGoalGpsPub;
    ros::Publisher _localPosePub;
    
    ros::ServiceClient _armingClient;
    ros::ServiceClient _setModeClient;

    // mux variables to control who send commands
    ros::ServiceClient _muxClient;
    std::string _muxManagerInput, _muxAvoidanceInput;
    
    
    void initialHovering();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
    void videoTimerCallback(const ros::TimerEvent& timer); 
    void managerTimerCallback(const ros::TimerEvent& timer); 
    void publish_x_y_z_yaw(double x, double y, double z, double new_yaw);
    void publishGoal(const Cart& goal);

    ros::Timer _videoTimer, _managerTimer;
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
    ros::Time _initTime, _preHoveringTime, _hoveringTime, _lastRequestTime, testBeginTime, testEndTime;
    std::string _mavName;
    
    geometry_msgs::PoseStamped _globalGoalMsg;
    geographic_msgs::GeoPoseStamped _globalGoalGpsMsg;

    //mavros states and commands
    mavros_msgs::State _mavState;
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    //used in code statistical functions
    void collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
    void linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr& msg);

    int _collisionNumber;
    double linearDist;
    double consumedEnergy, prevP;
    double _motorCoeff, _rotorVelSlowdown;
    ros::Time prevT;
    double travelledDistance;
    ros::Time initialCollisionTime;
    std::vector<Cart> _collisions;
    bool _saveSummary;
    std::ofstream _outputSummaryFile;
    std::string _summaryName;
        
    //used in graphical output
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
