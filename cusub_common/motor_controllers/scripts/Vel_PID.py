#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import tf
import time
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

"""
    Purpose of this file:

    Subscribes to estimates and republishes 
    - yaw
    - roll
    - pitch
    - depth
    - forward velocity
    - sideways velocity
    to the pid loop as the states so it can determine controls.

    - Luke Morrissey (luke.morrissey@colorado.edu)
"""


class Pose_PID():

    def __init__(self):
        ## Publish to PID yaw state
        self.yaw_pub = rospy.Publisher('cusub_common/motor_controllers/pid/yaw/state',Float64,queue_size=1)
        ## Publish to PID roll state
        self.roll_pub = rospy.Publisher('cusub_common/motor_controllers/pid/roll/state',Float64,queue_size=1)
        ## Publish to PID pitch state
        self.pitch_pub = rospy.Publisher('cusub_common/motor_controllers/pid/pitch/state',Float64,queue_size=1)

        self.depth_pub = rospy.Publisher('cusub_common/motor_controllers/pid/depth/state',Float64,queue_size=1)
        ## Publish to PID drive state
        self.drive_pub = rospy.Publisher('cusub_common/motor_controllers/pid/drive_vel/state',Float64,queue_size=1)

        ## Publish to PID strafe state
        self.strafe_pub = rospy.Publisher('cusub_common/motor_controllers/pid/strafe_vel/state',Float64,queue_size=1)

        self.state = rospy.Subscriber('cusub_common/odometry/filtered', Odometry, self.state_callback, queue_size=1)


    def state_callback(self, msg):
        '''translate setpoints callback function based on current and prev state of vehicle'''
        #postion variables
        position = msg.pose.pose.position

        #depth control
        depth = position.z

        #roll pitch control
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])

        #drive and strafe setpoints
        drive = msg.twist.twist.linear.x
        strafe = msg.twist.twist.linear.y

        self.setpoint_publisher(yaw, roll, pitch, depth, drive, strafe)


    def setpoint_publisher(self, yaw, roll, pitch, depth, drive, strafe):
        self.yaw_pub.publish(yaw)
        self.roll_pub.publish(roll)
        self.pitch_pub.publish(pitch)
        self.depth_pub.publish(depth)
        self.drive_pub.publish(drive)
        self.strafe_pub.publish(strafe)


##Main Function
def main():
    rospy.init_node('Pose_PID')
    m = Pose_PID()
    rospy.spin()

if __name__ == "__main__":
    main()
