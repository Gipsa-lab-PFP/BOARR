#include <cmath>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/ModelStates.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/ContactsState.h>
#include <topic_tools/MuxSelect.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "BenchmarkManager.hpp"
#include "util.hpp"

BenchmarkManager::BenchmarkManager() :
    _nh(ros::NodeHandle()),
    it(_nh)
{ 
    this->initParam();
    
    /*********** ROS PUBLISHER AND SUBSCRIBER CREATIONS ***************/
    // publishers that gives an input to the avoidance stack
    _globalGoalPub = _nh.advertise<geometry_msgs::PoseStamped>("command/goal", 1); 
    _globalGoalGpsPub = _nh.advertise<geographic_msgs::GeoPoseStamped>("command/goal_gps", 1); 
    _localPosePub = _nh.advertise<geometry_msgs::PoseStamped>("command/local_position", 10);
    
    // Subscriber that gives the position of all gazebo models. Used in order to draw a view from above of a simple scene when all the models are well known 
    _worldSub = _nh.subscribe("/gazebo/model_states", 1, &BenchmarkManager::gazeboWorldCallback, this ); 
    
    // Timer used by the manager
    _managerTimer = _nh.createTimer(ros::Duration(0.05), &BenchmarkManager::managerTimerCallback, this );

    // Callback on time, used to create the video at a defined frequency and to stop the tesf if it exceed a certain duration
    if (_showImg || _saveVideo) 
    {
	_videoTimer = _nh.createTimer(ros::Duration(1./_videoFreq), &BenchmarkManager::videoTimerCallback, this );
    }
    
    // Odometry Callback and main callback, send new goal when a goal is reached, update most of the benchmark performance criteria 
    _odomSub = _nh.subscribe("odometry", 1, &BenchmarkManager::odomCallback, this );

    // LinkState Callback, to compute the Energy Used during the flight
    _linkStatesSub = _nh.subscribe("/gazebo/link_states", 1, &BenchmarkManager::linkStatesCallback, this );

    // MAV state Callback
    _stateSub = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, &BenchmarkManager::stateCallback, this);
    
    // Collision Callback
    _collisionSub = _nh.subscribe("bumper_vals", 100, &BenchmarkManager::collisionCallback, this );
    
    // Image Callback for display
    _imgSub = it.subscribe( "raw_camera", 1, &BenchmarkManager::imgCallback, this); 
    
    // Mavros arming / setMode clients
    _armingClient = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _setModeClient = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // Mux client
    _muxClient = _nh.serviceClient<topic_tools::MuxSelect>("command_mux/select");
    
//     ros::Duration(.1).sleep(); // This sleep is also needed to let all the ros stuff initialize itself
  
//     // Send the first goal and initialize the test timer
//     geometry_msgs::Vector3Stamped next_goal;
//     next_goal.vector.x = _goals.at(_currentGoalNumber).x;
//     next_goal.vector.y = _goals.at(_currentGoalNumber).y;
//     next_goal.vector.z = _goals.at(_currentGoalNumber).z;
//     next_goal.header.stamp = ros::Time::now();
//     _globalGoalPub.publish(next_goal);
//     sensor_msgs::NavSatFix nextGoalGps;
//     nextGoalGps.latitude = 45.193100 - 9.00e-6 * _goals.at(_currentGoalNumber).x; //very loose approx of the link between gazebo coordinates and gps coordinates but OK considering the small motion 
//     nextGoalGps.longitude = 5.763788 + 1.2733e-5*_goals.at(_currentGoalNumber).y;
//     nextGoalGps.altitude = 210.0 + _goals.at(_currentGoalNumber).z;
//     nextGoalGps.header.stamp = ros::Time::now();
//     _globalGoalGpsPub.publish(nextGoalGps);

//     std::cout << "Benchmark Manager : Sending first goal :  x:" << _goals.at(_currentGoalNumber).x << " y:" << _goals.at(_currentGoalNumber).y << " z:"<< _goals.at(_currentGoalNumber).z << std::endl; 

//     testBeginTime = ros::Time::now();  
}

void BenchmarkManager::initParam()
{
    ros::NodeHandle nh_private_("~");
    
    /*********** TEST PARAMETERS, details in the Yaml Configuration Files and in the Shell Script ***************/
    _mavName = nh_private_.param<std::string>("mavName", "hummingbird");

    _testMaxTime = nh_private_.param<double>("testMaxTime", 60.0); 
    
    _initialHoverTODO = nh_private_.param<bool>("initialHovering/status", true);
    _initialHoverPos.x = nh_private_.param<double>("spawnPos/x", 0.0);
    _initialHoverPos.y = nh_private_.param<double>("spawnPos/y", 0.0);
    _initialHoverPos.z = nh_private_.param<double>("spawnPos/z", 1.0) + 2.0;
    _initialHoverYaw = nh_private_.param<double>("initialHovering/yaw", 0.0);
    _initialHoverDuration = nh_private_.param<double>("initialHovering/duration", 6.0); 

    _stopOnCollision = nh_private_.param<bool>("stopOnCollision",false);
    _finalLending = nh_private_.param<bool>("finalLending", true);

    _perfectTest = nh_private_.param<bool>("perfectTest",true);

    _controlType = nh_private_.param<int>("controlType", 0); 
    
    _muxManagerInput = nh_private_.param<std::string>("mux_manager_input", "benchmark_manager" );
    _muxAvoidanceInput = nh_private_.param<std::string>("mux_manager_input", "avoidance_controller" );
    
    _motorCoeff = nh_private_.param<double>("motor_coeff", 6e-5);
    _rotorVelSlowdown = nh_private_.param<double>("rotor_vel_slowdown", 10.);
    
    
    /*********** GOAL SETUP, those parameters are defined in the world.yaml file ***************/
    std::string goal_string = "goals/";
    unsigned int i = 1;
    while (nh_private_.hasParam(goal_string + std::to_string(i) + "/x" )) 
    {
        Cart tmp; 
        tmp.x = nh_private_.param<double>(goal_string + std::to_string(i) + "/x", 0.0); 
        tmp.y = nh_private_.param<double>(goal_string + std::to_string(i) + "/y", 0.0);
        tmp.z = nh_private_.param<double>(goal_string + std::to_string(i) + "/z", 1.0);
        _goals.push_back(tmp);
        double min_dist = nh_private_.param<double>(goal_string + std::to_string(i) + "/minDistToValidate", 0.1);
        _goalsMinDistToValidate.push_back(min_dist);
        i++;
    }
    
    /*********** OUTPUT PARAMETERS, defined in Yaml configuration File and in the Shell script ***************/
    _saveVideo = nh_private_.param<bool>("saveVideo", false); 
    _videoName = nh_private_.param<std::string>("videoName", "video.avi" );
    _videoFreq = nh_private_.param<double>("videoFreq", 20.0);

    _saveImage = nh_private_.param<bool>("saveImage", false); 
    _imageName = nh_private_.param<std::string>("imageName", "image.jpg" );

    _showImg = nh_private_.param<bool>("showLiveImage", false); 

    _saveSummary = nh_private_.param<bool>("saveSummary", false);
    _summaryName = nh_private_.param<std::string>("summaryName", "DefaultBenchmarkResults.txt");

    /*********** INTERNAL PARAMETERS INITIALIZATION ***************/
    _stop = false; // boolean which value changes to true when the test has ended
    
    _correctInitialization = false;

    // Store Counters used while drawing the output
    _currentGoalNumber = 0; // Store the current Goal ID

    initialCollisionTime = ros::Time::now(); // Store the last collision time in order to count a single collision when a contact appear in the last 100ms
    _collisionNumber = 0; // Store the collision number

    lastDroneDrownStamp = ros::Time::now(); // Store the odometry stamp of the last DROWN quadcopter on the output Image

    travelledDistance = 0; // Initialize the travelled distance
    consumedEnergy = 0; // Initialize the consumed energy
    linearDist = 0;
    testTime = 0;
    prevP = -1; // Init for energy consumption computation

    /*********** INITIALIZATIONS ***************/

    _worldCallback_CALLED = false;
    
    //setup of the video writer in full HD at the defined frequency. 
    if ( _saveVideo )
    {
        outputVideo.open(_videoName, CV_FOURCC('D','I','V','3'), _videoFreq, cv::Size(1920,1080), true);
        if (!outputVideo.isOpened())
        {
            std::cout  << "Benchmark Manager: Could not open the output video " <<  std::endl;
        }
    }
    
    if ( _saveSummary )
    {
        _outputSummaryFile.open(_summaryName.c_str(),std::ios_base::app | std::ios_base::out);
    }
    
    _globalGoalMsg.header.frame_id="world";
    _globalGoalMsg.pose.orientation.x=0.;
    _globalGoalMsg.pose.orientation.y=0.;
    _globalGoalMsg.pose.orientation.z=0.;
    _globalGoalMsg.pose.orientation.w=1.;
    
    _globalGoalGpsMsg.header.frame_id="world";
    _globalGoalGpsMsg.pose.orientation.x=0.;
    _globalGoalGpsMsg.pose.orientation.y=0.;
    _globalGoalGpsMsg.pose.orientation.z=0.;
    _globalGoalGpsMsg.pose.orientation.w=1.;

    _initTime = ros::Time::now();
    _managerState = SET_MUX;
}

void BenchmarkManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    switch(_managerState)
    {
    case HOVERING:
    {
	double disToHoverPos = (_initialHoverPos.x - msg->pose.pose.position.x ) * ( _initialHoverPos.x - msg->pose.pose.position.x ) 
	    + (_initialHoverPos.y - msg->pose.pose.position.y ) * ( _initialHoverPos.y - msg->pose.pose.position.y )  
	    + (_initialHoverPos.z - msg->pose.pose.position.z ) * ( _initialHoverPos.z - msg->pose.pose.position.z );
        if(disToHoverPos<0.1)
	{
	    ROS_INFO("Initial hover position reached, switch to TESTING mode");
	    ROS_INFO_STREAM("Benchmark Manager : Sending goal "<< _currentGoalNumber << " :    x:" << _goals.at(_currentGoalNumber).x << " y:" << _goals.at(_currentGoalNumber).y << " z:"<< _goals.at(_currentGoalNumber).z);

	    topic_tools::MuxSelect mux_select;
	    mux_select.request.topic = _muxAvoidanceInput;
	    bool mux_ok = false;
	    for(int k=0; k<3; k++)
	    {
		if( _muxClient.call(mux_select))
		{
		    ROS_DEBUG_STREAM("Mux switched to "<< _muxAvoidanceInput);
		    mux_ok=true;
		    break;
		}
	    }
	    if(mux_ok)
	    {
		testBeginTime = ros::Time::now();
		_correctInitialization = true;
		_managerState = TESTING;
	    }else{
		ROS_ERROR("BenchmarkManager : impossible to set mux");
		_correctInitialization = false;
		_stop=true;
		_managerState = FAILED;
	    }
	}
	break;
    }
    case TESTING:
    {
	// Update the Odometry
	Odometry odom;
	odom.pose.position.x = msg->pose.pose.position.x; 
	odom.pose.position.y = msg->pose.pose.position.y; 
	odom.pose.position.z = msg->pose.pose.position.z; 
	odom.pose.quaternion.w = msg->pose.pose.orientation.w;
	odom.pose.quaternion.x = msg->pose.pose.orientation.x;
	odom.pose.quaternion.y = msg->pose.pose.orientation.y;
	odom.pose.quaternion.z = msg->pose.pose.orientation.z;
	odom.twist.linear.x = msg->twist.twist.linear.x;
	odom.twist.linear.y = msg->twist.twist.linear.y;
	odom.twist.linear.z = msg->twist.twist.linear.z;
	odom.twist.angular.x = msg->twist.twist.angular.x;
	odom.twist.angular.y = msg->twist.twist.angular.y;
	odom.twist.angular.z = msg->twist.twist.angular.z;
	odom.stamp = msg->header.stamp;
	_odomVector.push_back(odom);
	
	// Depending on the position of the drone, send a new goal
	Point_3D goalPos(CART, _goals.at(_currentGoalNumber).x, _goals.at(_currentGoalNumber).y, _goals.at(_currentGoalNumber).z); 
	Point_3D dronePos(CART, odom.pose.position.x, odom.pose.position.y, odom.pose.position.z); 
	if ( goalPos.dist_to_point(dronePos) < _goalsMinDistToValidate.at(_currentGoalNumber))
	{
	    if ( _currentGoalNumber == _goals.size() - 1 )
	    {
		ROS_INFO("Last Goal Reached, stopping the node");
		testEndTime = ros::Time::now();
		_stop = true;
		_managerState = SUCCESSED;
	    }   
	    else 
	    {
		_currentGoalNumber ++;
		
		ROS_INFO_STREAM("Benchmark Manager : Sending goal "<< _currentGoalNumber << " :    x:" << _goals.at(_currentGoalNumber).x << " y:" << _goals.at(_currentGoalNumber).y << " z:"<< _goals.at(_currentGoalNumber).z);
	    }
	}
	
	// Update the performance criterias
	// Distance parcourue
	if ( _odomVector.size() > 1 )
	{
	    Point_3D oldDronePos(CART, _odomVector.at(_odomVector.size()-2).pose.position.x, _odomVector.at(_odomVector.size()-2).pose.position.y, _odomVector.at(_odomVector.size()-2).pose.position.z);
	    Point_3D newDronePos(CART, _odomVector.at(_odomVector.size()-1).pose.position.x, _odomVector.at(_odomVector.size()-1).pose.position.y, _odomVector.at(_odomVector.size()-1).pose.position.z);
	    travelledDistance += oldDronePos.dist_to_point(newDronePos);
	}
	
	// Distance parcourue sur le trajet
	double distBetweenPastWaypoints = 0;
	if ( _currentGoalNumber > 0 )
	{
	    Point_3D initialPos(CART, _initialHoverPos.x, _initialHoverPos.y, _initialHoverPos.z);
	    Point_3D firstWaypoint(CART, _goals.at(0).x, _goals.at(0).y, _goals.at(0).z);
	    distBetweenPastWaypoints += initialPos.dist_to_point(firstWaypoint);
	    
	    int id = 1;
	    while ( id < _currentGoalNumber )
	    {
		Point_3D idWaypoint(CART, _goals.at(id).x, _goals.at(id).y, _goals.at(id).z);
		Point_3D idMinusOneWaypoint(CART, _goals.at(id-1).x, _goals.at(id-1).y, _goals.at(id-1).z);
		distBetweenPastWaypoints+= idWaypoint.dist_to_point(idMinusOneWaypoint);
		id ++;
	    }
	}
	
	Point_3D currWaypoint(CART, _goals.at(_currentGoalNumber).x, _goals.at(_currentGoalNumber).y, _goals.at(_currentGoalNumber).z);
	Point_3D prevWaypoint;
	if ( _currentGoalNumber == 0 )
	{
	    prevWaypoint.define(CART, _initialHoverPos.x, _initialHoverPos.y, _initialHoverPos.z);
	}
	else
	{
	    prevWaypoint.define(CART, _goals.at(_currentGoalNumber-1).x, _goals.at(_currentGoalNumber-1).y, _goals.at(_currentGoalNumber-1).z);
	}
	double xd = currWaypoint.x() - prevWaypoint.x();
	double yd = currWaypoint.y() - prevWaypoint.y();
	double zd = currWaypoint.z() - prevWaypoint.z();
	double norm = std::sqrt(xd*xd+yd*yd+zd*zd);
	xd/=norm;
	yd/=norm;
	zd/=norm;
	double projX = (dronePos.x()-prevWaypoint.x())*xd;
	double projY = (dronePos.y()-prevWaypoint.y())*yd;
	double projZ = (dronePos.z()-prevWaypoint.z())*zd;
	
	Point_3D proj(CART, projX, projY, projZ);
	double linearDistTmp = distBetweenPastWaypoints + projX+projY+projZ;;
	if (linearDist < linearDistTmp) linearDist = linearDistTmp;
	if (linearDist > 1000) //if we sucessfully fly for more than 1km, then stop and validate ! 
	{
	    testEndTime = ros::Time::now();
	    _stop = true;
	    _managerState = SUCCESSED;
	}	
    }
    break;
    default:
	break;
    }
    
    return;
    
}

void BenchmarkManager::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	std::cout << "Benchmark Manager: Error during the depth callback" << std::endl; 
	return;
    }
    cv_ptr->image.copyTo(this->_currentImg);
    _camStamp = msg->header.stamp;
}

void BenchmarkManager::gazeboWorldCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    //XXX fine for perfect worlds, really loose approx of the place of the trees for the forests
    for ( int i = 0; i < (int) msg->pose.size() ; i++ )
    {
        if ( msg->name.at(i).find("mesh")!=std::string::npos || msg->name.at(i).find("ylin")!=std::string::npos ) //check if the item name has ylin included => cylinders=> or mesh "my_mesh" which is the name of the trees
        {
            _modelPos.push_back(Cart(msg->pose.at(i).position.x,msg->pose.at(i).position.y,msg->pose.at(i).position.z));
        }
    }
    
    _worldCallback_CALLED = true;
    //this callback should only be executed one time so : 
    _worldSub.shutdown();
}

void BenchmarkManager::collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
    if(_managerState != TESTING)
    {
	return;
    }
    
    if ( msg->states.size() != 0 && ros::Time::now().toSec() > initialCollisionTime.toSec() + 0.1)
    {
	ROS_INFO("Collision detected !");
        // store only the first contact position of the first state ( multiple contact can appear at the same time but they all show the same thing : a collision occured )
        this->_collisions.push_back( Cart( msg->states.at(0).contact_positions.at(0).x,msg->states.at(0).contact_positions.at(0).y,msg->states.at(0).contact_positions.at(0).z));
        initialCollisionTime = ros::Time::now();
        _collisionNumber ++;
        if ( _stopOnCollision )
    	{
	    ROS_INFO("Benchmark Manager: Collision Detected, End of the test");
	    testEndTime = ros::Time::now();
	    _stop = true;
    	}
    }
}

void BenchmarkManager::videoTimerCallback(const ros::TimerEvent& timer)
{
    createImg();
}

void BenchmarkManager::managerTimerCallback(const ros::TimerEvent& timer)
{  
    if(_initTime==ros::Time(0))
    {
	_initTime = timer.current_real;
    }

    switch(_managerState)
    {
    case SET_MUX:
    {
	if(timer.current_real-_initTime>ros::Duration(20))
	{
	    ROS_ERROR("init time too long");
	    _stop=true;
	    _correctInitialization = false;
	    _managerState = FAILED;
	    break;
	}
	topic_tools::MuxSelect mux_select;
	mux_select.request.topic = _muxManagerInput;
	if( _muxClient.call(mux_select))
	{
	    ROS_DEBUG_STREAM("Mux switched to "<< _muxManagerInput);
	    _managerState = WAITING_CONNECTION;
	}
    }
    break;
    case WAITING_CONNECTION:
	if(timer.current_real-_initTime>ros::Duration(20))
	{
	    ROS_ERROR("init time too long");
	    _stop=true;
	    _correctInitialization = false;
	    _managerState = FAILED;
	    break;
	}
	if(_mavState.connected)
	{
	    ROS_INFO("MAV CONNECTED");
	    _managerState = PRE_HOVERING;
	    _preHoveringTime = timer.current_real;
	    _lastRequestTime = timer.current_real;
	}
	break;
	
    case PRE_HOVERING:
	if(timer.current_real-_preHoveringTime>ros::Duration(20))
	{
	    ROS_ERROR("pre_hovering time too long");
	    _correctInitialization = false;
	    _stop = true;
	    _managerState = FAILED;
	    break;
	}
	
	publish_x_y_z_yaw(_initialHoverPos.x, _initialHoverPos.y , _initialHoverPos.z , _initialHoverYaw);
	if(timer.current_real-_preHoveringTime>ros::Duration(2.))
	{
	    if(_mavState.mode!="OFFBOARD" && (timer.current_real-_lastRequestTime > ros::Duration(2.)))
	    {
		ROS_INFO("SEND OFFBOARD COMMAND");
		mavros_msgs::SetMode offb_set_mode;
		offb_set_mode.request.custom_mode = "OFFBOARD";
		offb_set_mode.request.base_mode = 0;		
		if( _setModeClient.call(offb_set_mode) &&
		    offb_set_mode.response.mode_sent)
		{
		    ROS_INFO("Offboard enabled");
		}
		_lastRequestTime = timer.current_real;
	    }else if(!_mavState.armed && (timer.current_real-_lastRequestTime > ros::Duration(2.)))
	    {
		ROS_INFO("SEND ARM COMMAND");
		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = true;
		if( _armingClient.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
		_lastRequestTime = timer.current_real;
	    }
	}
	if(_mavState.mode=="OFFBOARD" && _mavState.armed)
	{
	    ROS_INFO("HOVERING");
	    _hoveringTime = timer.current_real;
	    _managerState = HOVERING;
	}
	break;
	
    case HOVERING:
	publish_x_y_z_yaw(_initialHoverPos.x, _initialHoverPos.y , _initialHoverPos.z , _initialHoverYaw);
	if(timer.current_real-_hoveringTime>ros::Duration(60))
	{
	    ROS_ERROR("Hovering time too long");
	    _correctInitialization = false;
	    _stop = true;
	    _managerState = FAILED;
	}
	break;
	
    case TESTING:
    {
	// Current Test time
	testTime = ros::Time::now().toSec() - testBeginTime.toSec();

	if(testTime > _testMaxTime)
	{
	    std::cout << "Benchmark Manager: Test manager and film creator stopped because the test length exceedeed the max length defined " << std::endl; 
	    testEndTime = timer.current_real;
	    _stop = true;
	    _managerState = FAILED;
	}else if(_currentGoalNumber < _goals.size()){
	    publishGoal(_goals[_currentGoalNumber]);
	}
    }
    break;
    default:
	break;
    }
}

void BenchmarkManager::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    _mavState = *msg;
}

void BenchmarkManager::linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr& msg)
{
    ros::Time curT(ros::Time::now());
    if(_rotor_links.empty())
    {
	for(int i=0; i<msg->name.size(); i++)
	{
	    if(msg->name[i].find("rotor_")!=std::string::npos)
	    {
		_rotor_links.push_back(i);
	    }
	}
    }
    
    if(_managerState!=TESTING || _rotor_links.empty())
	return;

    //compute Power for the current motor speeds
    
    double currentP = 0;
    for(int i=0; i< _rotor_links.size(); i++)
    {
	double vel = _rotorVelSlowdown*msg->twist[_rotor_links[i]].angular.z;
	currentP += _motorCoeff*(vel*vel);
    }
    
    if ( prevP != -1 ) // prevP = -1 => first callback call
    {
        consumedEnergy += ( curT.toSec() - prevT.toSec() ) * currentP /3600.; //from J to Wh
    }
    prevP = currentP;
    prevT = curT;
}

void BenchmarkManager::initialHovering()
{
    std::cout << "Benchmark Manager: Start of the initial hovering" << std::endl; 
    publish_x_y_z_yaw(_initialHoverPos.x, _initialHoverPos.y , _initialHoverPos.z , _initialHoverYaw);
    if ( _initialHoverDuration <= 0.1 ) 
    {
        
        std::cout << "Benchmark Manager: Hovering until getchar()=a completed" << std::endl; 
        while ( std::getchar() != 'a'){}
    }
    else 
    {
        ros::Duration(std::max(_initialHoverDuration,0.1)).sleep(); 
    }
    
    std::cout << "Benchmark Manager: End of the initial hovering set duration" << std::endl; 
}

bool BenchmarkManager::stop()
{
    return _stop;
}

void BenchmarkManager::finalize()
{
    if ( !_correctInitialization ) 
    {
	std::cout << "Benchmark Manager : Failed initialization, most probably either RotorS or Gazebo crashed. Stopping the manager." << std::endl; 
	return;
    }
    if ( _finalLending ) 
    {
        // send message to controller to go down to 10cm at the current position 
        if ( _odomVector.size() == 0 )
        {
            publish_x_y_z_yaw(0.0, 0.0, 0.1, 0.0);
        }
        else
        {
            publish_x_y_z_yaw(_odomVector.at(_odomVector.size()-1).pose.position.x, _odomVector.at(_odomVector.size()-1).pose.position.y, 0.1, 0.0);
        }
        //wait 2s that this point is reached
        ros::Duration(2).sleep();
    }

    if ( _saveImage || _saveVideo )
    {
        createImg(); 
    }

    if ( _saveSummary )
    {
        //save the summary 
        double test_time = testEndTime.toSec() - testBeginTime.toSec();
        int completion = 0;
	if ( test_time <= _testMaxTime && _collisions.size() == 0 ) completion = 1; // there are only 3 ways to stop and to write a line in the code : 
	// test time reached maxtime || collision => completion = 0,reached last goal || linearDist > 1000 => completion = 1 

        _outputSummaryFile << completion << " " << _collisions.size() << " " << travelledDistance << " " << test_time << " " << consumedEnergy << " " << linearDist << std::endl;
    }
}

void BenchmarkManager::publish_x_y_z_yaw(double x, double y, double z, double new_yaw)
{
    Quaternion q;
    q.fromYawPitchRoll(new_yaw, 0, 0);

    geometry_msgs::PoseStamped pose_msg; //in NWU
    pose_msg.header.stamp=ros::Time::now();
    
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation.x = q.x;
    pose_msg.pose.orientation.y = q.y;
    pose_msg.pose.orientation.z = q.z;
    pose_msg.pose.orientation.w = q.w;
    
    _localPosePub.publish(pose_msg);
}

void BenchmarkManager::publishGoal(const Cart& goal)
{
    ros::Time stamp(ros::Time::now());
    
    _globalGoalMsg.header.stamp = stamp;
    _globalGoalMsg.pose.position.x = goal.x;
    _globalGoalMsg.pose.position.y = goal.y;
    _globalGoalMsg.pose.position.z = goal.z;
    
    _globalGoalGpsMsg.header.stamp = stamp;
    _globalGoalGpsMsg.pose.position.latitude = 45.193100 + 9.00e-6 * goal.y;
    _globalGoalGpsMsg.pose.position.longitude = 5.763788 + 1.2733e-5 * goal.x;
    _globalGoalGpsMsg.pose.position.altitude = goal.z;

    _globalGoalPub.publish(_globalGoalMsg);
    _globalGoalGpsPub.publish(_globalGoalGpsMsg);
}
