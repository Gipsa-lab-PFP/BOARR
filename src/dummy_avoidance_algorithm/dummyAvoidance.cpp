#include <iostream>
#include <string>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h> // input
#include <sensor_msgs/NavSatFix.h> // input

#include <trajectory_msgs/MultiDOFJointTrajectory.h> // output

#include "dummyAvoidance.hpp"
#include "util.h"

DummyAvoidance::DummyAvoidance(ros::NodeHandle &nh)
{
    _globalgoal_DEFINED = false;
    _dronePos.define(CART, 0,0,0.1);

    // setup the callbacks & advertisers
    _odom_sub = nh.subscribe("odometry", 1, &DummyAvoidance::odomCallback, this );
    _globalGoalSub = nh.subscribe("command/global_goal", 1, &DummyAvoidance::globalGoalCallback, this );
    _trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/output_trajectory", 1);
}

DummyAvoidance::~DummyAvoidance(){}

void DummyAvoidance::globalGoalCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    //very loose approx of the link between gazebo coordinates and gps coordinates but OK considering the small motion of less than 100m compared to the earth radius
    _globalGoal.x = ( 45.193100 - msg->latitude ) / 9.00e-6;
    _globalGoal.y = ( msg->longitude - 5.763788 ) / 1.2733e-5;
    _globalGoal.z = msg->altitude -210.0;
 
    std::cout << " New goal: x: " << _globalGoal.x << " y : " << _globalGoal.y << " z : " << _globalGoal.z << std::endl; 
    std::cout << "Rotate in the direction of the new goal" << std::endl;
    Point_3D goal_HF(CART, _globalGoal.x - _dronePos.x(), _globalGoal.y - _dronePos.y(), _globalGoal.z - _dronePos.z());
    double new_yaw = goal_HF.theta();
    publish_x_y_z_yaw( _dronePos.x(), _dronePos.y(), _dronePos.z(), new_yaw );
    ros::Duration(5).sleep();
    std::cout << "Go Straight For the goal" << std::endl;

    _globalgoal_DEFINED = true;
    return;
}

void DummyAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // store the drone position in memory
    _dronePos.x(msg->pose.pose.position.x);
    _dronePos.y(msg->pose.pose.position.y);
    _dronePos.z(msg->pose.pose.position.z);

    // if a waypoint has been defined go in its direction if not return;
    if ( !_globalgoal_DEFINED ) return;
    Point_3D goal_diff(CART, _globalGoal.x - _dronePos.x(), _globalGoal.y - _dronePos.y(), _globalGoal.z - _dronePos.z());
    if ( goal_diff.rho() < 0.5 ) return;//Don't send new command if the drone is at the goal
    double new_yaw = goal_diff.theta(); //angle to the goal
    if ( goal_diff.rho() > 1.5 ) goal_diff.rho(1.5); //we at aim the point at 1.5m in the direction of the goal
    publish_x_y_z_yaw( _dronePos.x() + goal_diff.x(), _dronePos.y() + goal_diff.y(), _dronePos.z() + goal_diff.z(), new_yaw);
}

void DummyAvoidance::publish_x_y_z_yaw(double x,double y , double z , double new_yaw)
{
    Quaternion q;
    q.fromYawPitchRoll(new_yaw, 0, 0);//yaw, picth, roll

    // silent translation back of the waypoint in the world referentiel 
    geometry_msgs::Transform transform_msg;
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

    while (_trajectory_pub.getNumSubscribers() == 0 && ros::ok()) 
    {
        ROS_INFO("There is no subscriber available, trying again in 1 second.");
        ros::Duration(1.0).sleep();
    }

    _trajectory_pub.publish(trajectory_msg);
}

