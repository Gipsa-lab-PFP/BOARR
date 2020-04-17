#!/usr/bin/env python  
import rospy

import tf.transformations as transfo
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped

class odom2posetwist:
    def __init__(self):
        self.pos_noise = rospy.get_param("~pos_noise", 0.001)
        self.pos_sigma2 = self.pos_noise*self.pos_noise
        self.rot_noise = rospy.get_param("~rot_noise", 0.001)
        self.rot_sigma2 = self.rot_noise*self.rot_noise
        self.vel_noise = rospy.get_param("~vel_noise", 0.001)
        self.vel_sigma2 = self.vel_noise*self.vel_noise
        self.rotvel_noise = rospy.get_param("~rotvel_noise", 0.001)
        self.rotvel_sigma2 = self.rotvel_noise*self.rotvel_noise
        
        self.sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=10)
        self.pose_cov_pub = rospy.Publisher("pose_cov", PoseWithCovarianceStamped, queue_size=10)
        self.twist_pub = rospy.Publisher("speed_twist_cov", TwistWithCovarianceStamped, queue_size=10)
        
    def odomCallback(self, msg):
        #rospy.loginfo("odomCallback")
        pose = PoseStamped()
        pose_cov = PoseWithCovarianceStamped()
        twist = TwistWithCovarianceStamped()
        pose.header = msg.header
        #pose.pose = msg.pose.pose
        noise = np.random.normal(0,self.pos_noise, 3)
        pose.pose.position.x = msg.pose.pose.position.x + noise[0]
        pose.pose.position.y = msg.pose.pose.position.y + noise[1]
        pose.pose.position.z = msg.pose.pose.position.z + noise[2]

        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]

        noise = np.random.normal(0,self.rot_noise, 3)
        q_noise = transfo.quaternion_multiply(q, transfo.quaternion_from_euler(noise[0],
                                                                               noise[1],
                                                                               noise[2]))
        pose.pose.orientation.x = q_noise[0]
        pose.pose.orientation.y = q_noise[1]
        pose.pose.orientation.z = q_noise[2]
        pose.pose.orientation.w = q_noise[3]

        pose_cov.header = msg.header
        pose_cov.pose.pose = pose.pose
        
        pose_cov.pose.covariance=[self.pos_sigma2, 0, 0, 0, 0, 0,
                                  0, self.pos_sigma2, 0, 0, 0, 0,
                                  0, 0, self.pos_sigma2, 0, 0, 0,
                                  0, 0, 0, self.rot_sigma2, 0, 0,
                                  0, 0, 0, 0, self.rot_sigma2, 0,
                                  0, 0, 0, 0, 0, self.rot_sigma2]
        
        
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

        noise = np.random.normal(0,self.vel_noise, 3)
        
        twist.twist.twist.linear.x=v2[0] + noise[0]
        twist.twist.twist.linear.y=v2[1] + noise[1]
        twist.twist.twist.linear.z=v2[2] + noise[2]

        noise = np.random.normal(0,self.rotvel_noise, 3)
        twist.twist.twist.angular.x=vrot2[0] + noise[0]
        twist.twist.twist.angular.y=vrot2[1] + noise[1]
        twist.twist.twist.angular.z=vrot2[2] + noise[2]
        
        twist.twist.covariance=[self.vel_sigma2, 0, 0, 0, 0, 0,
                                0, self.vel_sigma2, 0, 0, 0, 0,
                                0, 0, self.vel_sigma2, 0, 0, 0,
                                0, 0, 0, self.rotvel_sigma2, 0, 0,
                                0, 0, 0, 0, self.rotvel_sigma2, 0,
                                0, 0, 0, 0, 0, self.rotvel_sigma2]
        
        self.pose_pub.publish(pose)            
        self.pose_cov_pub.publish(pose_cov)
        self.twist_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('odom2posetwist')
    o2pt = odom2posetwist()
    rospy.spin()

