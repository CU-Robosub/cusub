#!/usr/bin/env python
import rospy
import numpy as np
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped


def normalize_ang(a):
    while a > np.pi:
        a -= 2*np.pi
    while a < -np.pi:
        a+= 2*np.pi 
    return a

class Convert:
    def __init__(self):
        rospy.Subscriber("/stereo_odometer/odometry",Odometry,self.odom_callback)

        self.twist_pub = rospy.Publisher("/leviathan/description/stereo_odometer/twist",TwistWithCovarianceStamped,queue_size=10)

    def odom_callback(self,msg):
        speed = np.linalg.norm([msg.twist.twist.linear.x,msg.twist.twist.linear.y])
        if speed < 0.00001:
            return
        (r, p, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        direction = np.arctan2(msg.twist.twist.linear.y,msg.twist.twist.linear.x)
        direction -= yaw
        direction = normalize_ang(direction)

        adj_x_vel = speed * np.cos(direction)
        adj_y_vel = speed * np.sin(direction)

        tw = TwistWithCovarianceStamped()
        tw.header = msg.header
        tw.twist.covariance = msg.twist.covariance
        # for i in range(len(tw.twist.covariance)):
        #     tw.twist.covariance[i]*=2
        tw.twist.twist.linear.x = adj_x_vel
        tw.twist.twist.linear.y = adj_y_vel

        self.twist_pub.publish(tw)

        
if __name__ == "__main__":
    rospy.init_node("speed_converter")
    c = Convert()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()