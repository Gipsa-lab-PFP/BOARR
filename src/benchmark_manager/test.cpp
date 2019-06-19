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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gazebo_msgs/ContactsState.h>
#include <sensor_msgs/NavSatFix.h>
#include <topic_tools/MuxSelect.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "test.hpp"
#include "util.hpp"

Test::Test() :
  _nh(ros::NodeHandle()),
  it(_nh)
{ 
    this->initParam();

    /*********** ROS PUBLISHER AND SUBSCRIBER CREATIONS ***************/
    // publishers that gives an input to the avoidance stack
    _globalGoalPub = _nh.advertise<geometry_msgs::Vector3Stamped>("command/global_goal_gazebo_ref", 1); 
    _globalGoalGpsPub = _nh.advertise<sensor_msgs::NavSatFix>("command/global_goal", 1); 

    // publisher that's directly link to the position controller
    _trajectoryPub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/output_trajectory", 1); 

    // service to switch control type between position ( in this node ) and other mode if needed in tested algorithm
    _controlMuxClient = _nh.serviceClient<topic_tools::MuxSelect>("/"+_mavName+"/mux/select");

    // starting the hovering very early THEN starting the subscribers to start recording data only AFTER the hovering
    ros::Duration(.1).sleep(); // This sleep is needed to let all the ros stuff initialize itself ( neither the hovering or simply the ROS::Clock are working otherwise )
    if ( _initialHoverTODO ) 
    {
        this->initialHovering();
    }

    // Subscriber that gives the position of all gazebo models. Used in order to draw a view from above of a simple scene when all the models are well known 
    _worldSub = _nh.subscribe("/gazebo/model_states", 1, &Test::gazeboWorldCallback, this ); 

    // Callback on time, used to create the video at a defined frequency and to stop the tesf if it exceed a certain duration
    _timer = _nh.createTimer(ros::Duration(1./_videoFreq), &Test::timerCallback, this );

    // Odometry Callback and main callback, send new goal when a goal is reached, update most of the benchmark performance criteria 
    _odomSub = _nh.subscribe("odometry", 1, &Test::odomCallback, this );

    // Motor Callback, to compute the Energy Used during the flight
    _motorSub = _nh.subscribe("motor_speed", 1, &Test::motorCallback, this );

    // Collision Callback
    _collisionSub = _nh.subscribe("bumper_vals", 100000, &Test::collisionCallback, this );
    
    // Image Callback for display
    _imgSub = it.subscribe( "raw_camera", 1, &Test::imgCallback, this); 

    ros::Duration(.1).sleep(); // This sleep is also needed to let all the ros stuff initialize itself
  
    // Send the first goal and initialize the test timer
    geometry_msgs::Vector3Stamped next_goal;
    next_goal.vector.x = _goals.at(_currentGoalNumber).x;
    next_goal.vector.y = _goals.at(_currentGoalNumber).y;
    next_goal.vector.z = _goals.at(_currentGoalNumber).z;
    next_goal.header.stamp = ros::Time::now();
    _globalGoalPub.publish(next_goal);
    sensor_msgs::NavSatFix nextGoalGps;
    nextGoalGps.latitude = 45.193100 - 9.00e-6 * _goals.at(_currentGoalNumber).x; //very loose approx of the link between gazebo coordinates and gps coordinates but OK considering the small motion 
    nextGoalGps.longitude = 5.763788 + 1.2733e-5*_goals.at(_currentGoalNumber).y;
    nextGoalGps.altitude = 210.0 + _goals.at(_currentGoalNumber).z;
    nextGoalGps.header.stamp = ros::Time::now();
    _globalGoalGpsPub.publish(nextGoalGps);

    std::cout << "Benchmark Manager : Sending first goal :  x:" << _goals.at(_currentGoalNumber).x << " y:" << _goals.at(_currentGoalNumber).y << " z:"<< _goals.at(_currentGoalNumber).z << std::endl; 
    if ( _controlType == 0 )
    {
        std::cout << "Benchmark Manager : Test with control in position" << std::endl; 
    }
    else if ( _controlType == 1 )
    { 
        topic_tools::MuxSelect srv;
        srv.request.topic = "/"+_mavName+"/command/motor_speed_from_rpyrt";
        _controlMuxClient.call(srv);
        std::cout << "Benchmark Manager : Test with control in attitude" << std::endl; 
    }
    else if ( _controlType == 2 ) 
    {
        topic_tools::MuxSelect srv;
        srv.request.topic = "/"+_mavName+"/command/motor_speed_direct";
        _controlMuxClient.call(srv);
        std::cout << "Benchmark Manager : Test with control in motor Speed" << std::endl; 
    }
    else 
    {
        std::cout << "Benchmark Manager : Warning, unknown control type, staying with a control in position" << std::endl; 
    }

    testBeginTime = ros::Time::now();  
}

void Test::initParam()
{
    ros::NodeHandle nh_private_("~");
    
    /*********** TEST PARAMETERS, details in the Yaml Configuration Files and in the Shell Script ***************/
    _mavName = nh_private_.param<std::string>("mavName", "hummingbird");

    _testMaxTime = nh_private_.param<double>("testMaxTime", 60.0); 
    
    _initialHoverTODO = nh_private_.param<bool>("initialHovering/status", true);
    _initialHoverPos.x = _nh.param<double>("spawnX", 0.0);
    _initialHoverPos.y = _nh.param<double>("spawnY", 0.0);
    _initialHoverPos.z = _nh.param<double>("spawnZ", 1.0) + 2.0;
    _initialHoverYaw = nh_private_.param<double>("initialHovering/yaw", 0.0);
    _initialHoverDuration = nh_private_.param<double>("initialHovering/duration", 6.0); 

    _stopOnCollision = nh_private_.param<bool>("stopOnCollision",false);
    _finalLending = nh_private_.param<bool>("finalLending", true);

    _perfectTest = nh_private_.param<bool>("perfectTest",true);

    _controlType = nh_private_.param<int>("controlType", 0); 
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

}

void Test::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // The odometry callback is started when the simulated drone finish is hover. 
    // If all the initialization worked, the drone should be really close to the hover position
    // if its not the case, we count it as a failed initialization and close the program with an error code
    if ( _initialHoverTODO && !_correctInitialization )  
    {
        double disToHoverPos = (_initialHoverPos.x - msg->pose.pose.position.x ) * ( _initialHoverPos.x - msg->pose.pose.position.x ) 
                              + (_initialHoverPos.y - msg->pose.pose.position.y ) * ( _initialHoverPos.y - msg->pose.pose.position.y )  
                              + (_initialHoverPos.z - msg->pose.pose.position.z ) * ( _initialHoverPos.z - msg->pose.pose.position.z );
        // Compute dist to Hover Pos
        if ( disToHoverPos > 1.0 )
        {
            _stop = true;
        }
        else 
        {
            _correctInitialization = true;
        }
    }

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
            if( !_stop ) std::cout << "Benchmark Manager: Last Goal Reached, stopping the node" << std::endl;

            testEndTime = ros::Time::now();
            _stop = true;
        }
        else 
        {
            _currentGoalNumber ++;

            geometry_msgs::Vector3Stamped nextGoal;
            nextGoal.vector.x = _goals.at(_currentGoalNumber).x;
            nextGoal.vector.y = _goals.at(_currentGoalNumber).y;
            nextGoal.vector.z = _goals.at(_currentGoalNumber).z;
            nextGoal.header.stamp = ros::Time::now();
            _globalGoalPub.publish(nextGoal);
            sensor_msgs::NavSatFix nextGoalGps;
            nextGoalGps.latitude = 45.193100 - 9.00e-6 * _goals.at(_currentGoalNumber).x; //very loose approx of the link between gazebo coordinates and gps coordinates but OK considering the small motion 
            nextGoalGps.longitude = 5.763788 + 1.2733e-5*_goals.at(_currentGoalNumber).y;
            nextGoalGps.altitude = 210.0 + _goals.at(_currentGoalNumber).z;
            nextGoalGps.header.stamp = ros::Time::now();
            _globalGoalGpsPub.publish(nextGoalGps);

            std::cout << "Benchmark Manager : Sending goal n°"<< _currentGoalNumber << " :    x:" << _goals.at(_currentGoalNumber).x << " y:" << _goals.at(_currentGoalNumber).y << " z:"<< _goals.at(_currentGoalNumber).z << std::endl; 
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
	}

    // Current Test time
    testTime = ros::Time::now().toSec() - testBeginTime.toSec();
}

void Test::imgCallback(const sensor_msgs::ImageConstPtr& msg)
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

void Test::gazeboWorldCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    //XXX fine for perfect worlds, really loose approx of the place of the trees for the forests
    for ( int i = 0; i < (int) msg->pose.size() ; i++ )
    {
        if ( msg->name.at(i).find("mesh")!=std::string::npos || msg->name.at(i).find("ylin")!=std::string::npos ) //check if the item name has ylin included => cylinders=> or mesh "my_mesh" which is the name of the trees
        {
            this->_modelPos.push_back(Cart(msg->pose.at(i).position.x,msg->pose.at(i).position.y,msg->pose.at(i).position.z));
        }
    }
    _worldCallback_CALLED = true;
    //this callback should only be executed one time so : 
    _worldSub.shutdown();
}

void Test::collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
    if ( msg->states.size() != 0 && ros::Time::now().toSec() > initialCollisionTime.toSec() + 0.1)
    {
        // store only the first contact position of the first state ( multiple contact can appear at the same time but they all show the same thing : a collision occured )
        this->_collisions.push_back( Cart( msg->states.at(0).contact_positions.at(0).x,msg->states.at(0).contact_positions.at(0).y,msg->states.at(0).contact_positions.at(0).z));
        initialCollisionTime = ros::Time::now();
        _collisionNumber ++;
        if ( _stopOnCollision )
    	{
    		std::cout << "Benchmark Manager: Collision Detected, End of the test" << std::endl;
    		_stop = true;
    	}
    }
}

void Test::timerCallback(const ros::TimerEvent& timer)
{
    // if needed, generate the image
    if (_showImg || _saveVideo) 
    {
        createImg();
    }

    float loop_total_time =  ros::Time::now().toSec() - testBeginTime.toSec() ;
    if ( fmod(loop_total_time, 5.0) < 0.051 )
    {
        std::cout << "Benchmark Manager: The test started : " << loop_total_time << "s ago, max time : " <<  _testMaxTime << "s" << std::endl;
    }
    if ( loop_total_time > _testMaxTime )
    {
        std::cout << "Benchmark Manager: Test manager and film creator stopped because the test length exceedeed the max length defined " << std::endl; 
        testEndTime = ros::Time::now();
        _stop = true;
    }
}

void Test::motorCallback(const mav_msgs::ActuatorsConstPtr& msg)
{
    //compute Power for the current motor speeds
    double currentP = 0;
    for ( int i= 0; i < 4 ; i ++ )
    {
        currentP += 2.13e-7 * std::pow(std::abs(msg->angular_velocities[i]),3) 
                     + 1.75e-4 * std::pow(std::abs(msg->angular_velocities[i]),2) 
                     - 0.0187 * std::pow(std::abs(msg->angular_velocities[i]),1)
                     + 1.314;
    }
    
    if ( prevP != -1 ) // prevP = -1 => first callback call
    {
        consumedEnergy += ( msg->header.stamp.toSec() - prevT.toSec() ) * currentP /3600.; //from J to Wh
    }
    prevP = currentP;
    prevT = msg->header.stamp;
}

void Test::initialHovering()
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

bool Test::stop()
{
    return _stop;
}

void Test::finalize()
{
	if ( !_correctInitialization ) 
	{
	    std::cout << "Benchmark Manager : Failed initialization, most probably either RotorS or Gazebo crashed. Stopping the manager." << std::endl; 
		return;
	}
    if ( _finalLending ) 
    {
        // switching back to a control in position 
        topic_tools::MuxSelect srv;
        srv.request.topic = "/"+_mavName+"/command/motor_speed_from_leepos";
        _controlMuxClient.call(srv);
        std::cout << "Benchmark Manager : End of the test, taking back the control in position" << std::endl; 

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
        if ( test_time <= _testMaxTime && _collisions.size() == 0 ) completion = 1; // there are only 4 ways to stop and to write a line in the code :
                                                              // test time reached maxtime or collision => completion = 0, reached last goal or linearDist > 1000 => completion = 1
        _outputSummaryFile << completion << " " << _collisions.size() << " " << travelledDistance << " " << test_time << " " << consumedEnergy << " " << linearDist << std::endl;
    }
}

void Test::publish_x_y_z_yaw(double x, double y, double z, double new_yaw)
{
    Quaternion q;
    q.fromYawPitchRoll(new_yaw, 0, 0);

    geometry_msgs::Transform transform_msg; //in NWU
    transform_msg.translation.x = x;
    transform_msg.translation.y = y;
    transform_msg.translation.z = z;
    transform_msg.rotation.x = q.x;
    transform_msg.rotation.y = q.y;
    transform_msg.rotation.z = q.z;
    transform_msg.rotation.w = q.w;

    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    point_msg.transforms.push_back(transform_msg);

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.joint_names.push_back("base_link");
    trajectory_msg.points.push_back(point_msg);

    while (_trajectoryPub.getNumSubscribers() == 0 && ros::ok()) 
    {
        ROS_INFO("Benchmark Manager: There is no subscriber available, trying again in 1 second");
        ros::Duration(1.0).sleep();
    }
    _trajectoryPub.publish(trajectory_msg);
}
