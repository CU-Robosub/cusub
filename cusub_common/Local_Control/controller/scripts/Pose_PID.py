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

## @package Pose_PID
# @brief This package translates Pose control efforts to PID states
#
# @author "Cody Constine"

## Pose_PID
# @brief This is the main class of the modules, contains all the Pubs and Subs
# @ingroup Local_Control
#
class Pose_PID():

    def __init__(self):

        self.namespace = rospy.get_param("~namespace")

        ## Last X position
        self.last_x = 0
        ## Last Y position
        self.last_y = 0
        ## Last Z position
        self.last_z = 0
        ## Last time
        self.last_time = 0
        ## For initial setup
        self.first = True
    	self.last_pose = Odometry()
        ## This listens to transforms from tf2 package for Odometry
        self.listener = tf.TransformListener();
        ## Publish to PID yaw state
        self.yaw_pub_data = Float64()
        self.yaw_pub = rospy.Publisher(self.namespace + '/local_control/pid/yaw/state',Float64,queue_size=1)
        ## Publish to PID roll state
        self.roll_pub_data = Float64()
        self.roll_pub = rospy.Publisher(self.namespace + '/local_control/pid/roll/state',Float64,queue_size=1)
        ## Publish to PID pitch state
        self.pitch_pub_data = Float64()
        self.pitch_pub = rospy.Publisher(self.namespace + '/local_control/pid/pitch/state',Float64,queue_size=1)

        self.depth_pub_data = Float64()
        self.depth_pub = rospy.Publisher(self.namespace + '/local_control/pid/depth/state',Float64,queue_size=1)
        ## Publish to PID drive state
        self.drive_sum = 0
        self.drive_pub = rospy.Publisher(self.namespace + '/local_control/pid/drive/state',Float64,queue_size=1)

        ## Publish to PID strafe state

        self.strafe_sum = 0
        self.strafe_pub = rospy.Publisher(self.namespace + '/local_control/pid/strafe/state',Float64,queue_size=1)

        self.last_yaw = 0

        self.state = rospy.Subscriber('/sensor_fusion/odometry/filtered', Odometry, self.state_callback, queue_size=1)


    def state_callback(self, msg):
        '''translate setpoints callback function based on current and prev state of vehicle'''
        #postion variables
        prev_position = self.last_pose.pose.pose.position
        position = msg.pose.pose.position

        #depth control
        depth = position.z

        #roll pitch control
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])
        #drive and strafe setpoints
        (drive, strafe ) = self.point_and_shoot_model(position.x, prev_position.x,
                                                      position.y, prev_position.y,
                                                      yaw)

        self.setpoint_publisher(yaw, roll, pitch, depth, drive, strafe)
        self.last_pose = msg


    def point_and_shoot_model(self, x_curr, x_prev, y_curr, y_prev, yaw):
        ''' point and shoot model based of accumulation of x and y positions'''
        dx = x_curr - x_prev
        dy = y_curr - y_prev

        self.drive_sum = self.drive_sum +dx*math.cos(yaw)+dy*math.sin(yaw)
        self.strafe_sum = self.strafe_sum + dx*math.sin(yaw) - dy*math.cos(yaw)

        return self.drive_sum, self.strafe_sum


    def unicylce_model():
        ''' under construction, based of a differential drive model'''
        pass

    def setpoint_publisher(self, yaw, roll, pitch, depth, drive, strafe):
        self.yaw_pub.publish(yaw)
        self.roll_pub.publish(roll)
        self.pitch_pub.publish(pitch)
        self.depth_pub.publish(depth)
        self.drive_pub.publish(drive)
        self.strafe_pub.publish(strafe)



    ## Depth Callback
    # @param self The object pointer
    # @param msg The message returned by subscriber
    def depth_callback(self,msg):
        self.depth_pub_data = msg.pose.pose.position.z
        self.depth_pub.publish(self.depth_pub_data)
    def imu_callback(self,msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        #This is so we can turn to 180
        if((self.last_yaw*y)<0 and abs(self.last_yaw)>math.pi/2):
            y = np.sign(self.last_yaw)*(2*math.pi-abs(y))
        else:
            self.last_yaw = y

        #
        self.yaw_pub_data = y
        self.yaw_pub.publish(self.yaw_pub_data)
    ## Pose Callback
    #  This function takes the current pose and splits that out to the PID states
    # @param self The object pointer
    # @param msg The message returned by subscriber
    def pose_callback(self,msg):
        #This temp is constructed to be passes to tf and get the sub frame transforms
        temp = PoseStamped()
        temp.header = msg.header
        temp.pose = msg.pose.pose
        temp.header.stamp = rospy.Time(0)
        try:
            self.trans = self.listener.waitForTransform('/odom','/base_link',rospy.Time(0),rospy.Duration(.1))
            self.trans = self.listener.lookupTransform('/odom','/base_link',rospy.Time(0))
        except Exception as e:
            return
        #Next we turn the quaternions into euler angles
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        #This is so we can turn to 180
        if((self.last_yaw*y)<0 and abs(self.last_yaw)>math.pi/2):
            y = np.sign(self.last_yaw)*(2*math.pi-abs(y))
        else:
            self.last_yaw = y

        #
        self.yaw_pub_data = y
        self.yaw_pub.publish(self.yaw_pub_data)

        self.roll_pub_data = r
        self.roll_pub.publish(self.roll_pub_data)

        self.pitch_pub_data = p
        self.pitch_pub.publish(self.pitch_pub_data)
        #Here is where we use the sub frame to transform drive and strafe states
        (r, p, y) = tf.transformations.euler_from_quaternion(self.trans[1])
        #Calculate the transfroms
        dx = msg.pose.pose.position.x-self.last_pose.pose.pose.position.x
        dy = msg.pose.pose.position.y-self.last_pose.pose.pose.position.y

        self.drive_pub_data = self.drive_pub_data+dx*math.cos(y)+dy*math.sin(y)
        self.drive_pub.publish(self.drive_pub_data)

        self.strafe_pub_data = self.strafe_pub_data-dx*math.sin(y)+dy*math.cos(y)
        self.strafe_pub.publish(self.strafe_pub_data)
        self.last_pose = msg

##Main Function
def main():
    rospy.init_node('Pose_PID')
    m = Pose_PID()
    rospy.spin()

if __name__ == "__main__":
    main()
