#!/usr/bin/env python  
import rospy

import tf.transformations as transfo
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped

class odom2posetwist:
    def __init__(self):
        self.sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=10)
        self.pose_cov_pub = rospy.Publisher("pose_cov", PoseWithCovarianceStamped, queue_size=10)
        self.twist_pub = rospy.Publisher("speed_twist_cov", TwistWithCovarianceStamped, queue_size=10)
        
    def odomCallback(self, msg):
        pose = PoseStamped()
        pose_cov = PoseWithCovarianceStamped()
        twist = TwistWithCovarianceStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose_cov.header = msg.header
        pose_cov.pose = msg.pose
        
        if pose_cov.pose.covariance[0]==0:
            pose_cov.pose.covariance=[0.0001, 0, 0, 0, 0, 0,
                                  0, 0.0001, 0, 0, 0, 0,
                                  0, 0, 0.0001, 0, 0, 0,
                                  0, 0, 0, 0.0001, 0, 0,
                                  0, 0, 0, 0, 0.0001, 0,
                                  0, 0, 0, 0, 0, 0.0001]

        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        
        R = transfo.quaternion_matrix(transfo.quaternion_inverse(q))
        v = [msg.twist.twist.linear.x,
             msg.twist.twist.linear.y,
             msg.twist.twist.linear.z,
             1]
        vrot = [msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
                0]
        
        v2 = np.dot(R,v)
        vrot2 = np.dot(R, vrot)
        
        twist.header = msg.header
        twist.twist = msg.twist

        twist.twist.twist.linear.x=v2[0]
        twist.twist.twist.linear.y=v2[1]
        twist.twist.twist.linear.z=v2[2]
        twist.twist.twist.angular.x=vrot2[0]
        twist.twist.twist.angular.y=vrot2[1]
        twist.twist.twist.angular.z=vrot2[2]
        
        if twist.twist.covariance[0]==0:
            twist.twist.covariance=[0.0001, 0, 0, 0, 0, 0,
                                  0, 0.0001, 0, 0, 0, 0,
                                  0, 0, 0.0001, 0, 0, 0,
                                  0, 0, 0, 0.0001, 0, 0,
                                  0, 0, 0, 0, 0.0001, 0,
                                  0, 0, 0, 0, 0, 0.0001]

        self.pose_pub.publish(pose)            
        self.pose_cov_pub.publish(pose_cov)
        self.twist_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('odom2posetwist')
    o2pt = odom2posetwist()
    rospy.spin()

