#ifndef __DUMMY_AVOIDANCE_HPP__
#define __DUMMY_AVOIDANCE_HPP__

#include <ros/ros.h>

#include <nav_msgs/Odometry.h> // input
#include <sensor_msgs/NavSatFix.h> // input

#include <trajectory_msgs/MultiDOFJointTrajectory.h> // output

#include "util.h"

class DummyAvoidance
{
    protected:
        ros::Publisher _trajectory_pub;
        ros::Subscriber _odom_sub,_globalGoalSub;
        Cart _globalGoal;
        bool _globalgoal_DEFINED;
        Point_3D _dronePos;

        void globalGoalCallback(const sensor_msgs::NavSatFixConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void publish_x_y_z_yaw(double x,double y , double z , double yaw);

    public:
        DummyAvoidance(ros::NodeHandle &nh);
        ~DummyAvoidance();
}; 

#endif // __DUMMY_AVOIDANCE_HPP__
