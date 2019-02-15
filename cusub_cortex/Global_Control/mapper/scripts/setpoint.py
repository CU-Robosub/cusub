#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import tf
import time
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
## @package Pose_PID
#
# This is a package that takes the Pose and parse it to the PID controllers

## Documentation for a class.
#
# This is the main handler class
class Setpoint():

    def __init__(self):
        self.yaw_sub = rospy.Subscriber('/local_control/pid/yaw/setpoint', Float64, self.yaw_callback, queue_size=20)
        self.roll_sub = rospy.Subscriber('/local_control/pid/roll/setpoint', Float64, self.roll_callback, queue_size=20)
        self.pitch_sub = rospy.Subscriber('/local_control/pid/pitch/setpoint', Float64, self.pitch_callback, queue_size=20)
        self.depth_sub = rospy.Subscriber('/local_control/pid/depth/setpoint', Float64, self.depth_callback, queue_size=20)
        self.drive_sub = rospy.Subscriber('/local_control/pid/drive/setpoint', Float64, self.drive_callback, queue_size=20)
        self.strafe_sub = rospy.Subscriber('/local_control/pid/strafe/setpoint', Float64, self.strafe_callback, queue_size=20)

        self.yaw_setpoint_pub = rospy.Publisher('/local_control/pid/yaw/set',Float64,queue_size=20)
        self.yaw_setpoint_pub_data = Float64()
        self.roll_setpoint_pub = rospy.Publisher('/local_control/pid/roll/set',Float64,queue_size=20)
        self.roll_setpoint_pub_data = Float64()
        self.pitch_setpoint_pub = rospy.Publisher('/local_control/pid/pitch/set',Float64,queue_size=20)
        self.pitch_setpoint_pub_data = Float64()
        self.depth_setpoint_pub = rospy.Publisher('/local_control/pid/depth/set',Float64,queue_size=20)
        self.depth_setpoint_pub_data = Float64()
        self.drive_setpoint_pub = rospy.Publisher('/local_control/pid/drive/set',Float64,queue_size=20)
        self.drive_setpoint_pub_data = Float64()
        self.strafe_setpoint_pub = rospy.Publisher('/local_control/pid/strafe/set',Float64,queue_size=20)
        self.strafe_setpoint_pub_data = Float64()
        # Initial Values
        self.yaw_setpoint_pub_data = 0
        self.roll_setpoint_pub_data = 0
        self.pitch_setpoint_pub_data = 0
        self.depth_setpoint_pub_data = -.2
        self.drive_setpoint_pub_data = 0
        self.strafe_setpoint_pub_data = 0

    def yaw_callback(self, msg):
        self.yaw_setpoint_pub_data = msg.data
    def roll_callback(self, msg):
        self.roll_setpoint_pub_data = msg.data
    def pitch_callback(self, msg):
        self.pitch_setpoint_pub_data = msg.data
    def depth_callback(self, msg):
        self.depth_setpoint_pub_data = msg.data
    def drive_callback(self, msg):
        self.drive_setpoint_pub_data = self.drive_setpoint_pub_data+msg.data
    def strafe_callback(self, msg):
        self.strafe_setpoint_pub_data = self.strafe_setpoint_pub_data+msg.data


def main():
    rospy.init_node('Setpoint')
    m = Setpoint()
    r = rospy.Rate(30)
    # time.sleep(10)
    while not rospy.is_shutdown():
        m.yaw_setpoint_pub.publish(m.yaw_setpoint_pub_data)
        m.roll_setpoint_pub.publish(m.roll_setpoint_pub_data)
        m.pitch_setpoint_pub.publish(m.pitch_setpoint_pub_data)
        m.depth_setpoint_pub.publish(m.depth_setpoint_pub_data)
        m.drive_setpoint_pub.publish(m.drive_setpoint_pub_data)
        m.strafe_setpoint_pub.publish(m.strafe_setpoint_pub_data)
        r.sleep()

if __name__ == "__main__":
    main()
