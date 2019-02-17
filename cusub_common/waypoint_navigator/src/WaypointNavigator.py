#!/usr/bin/python2

import rospy
import tf
import math

from enum import Enum

from waypoint_navigator.srv import *

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

YAW_MODE = 1
STRAFE_MODE = 2

class WaypointNavigator(object):

    def __init__(self):

        self.waypointlist = []

        self.waypoint = None

        self.movement_mode = STRAFE_MODE
        self.waypoint_yaw = 0

    def addWaypoint(self, req):
        rospy.loginfo(req.waypoint)
        self.waypointlist.append(req.waypoint)
        if(self.waypoint is None):
            self.waypoint = self.waypointlist[0]
        return []

    def advanceWaypoint(self):
        rospy.loginfo("Waypoint Reached!")
        if(len(self.waypointlist) > 0):
            self.waypoint = self.waypointlist[0]

    def odometryCallback(self, odom):

        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])

        if(self.waypoint is not None):

            currentwaypoint = self.waypoint

            dx = currentwaypoint.x - position.x
            dy = currentwaypoint.y - position.y
            dz = currentwaypoint.z - position.z

            xy_dist = math.sqrt(dx**2 + dy**2)
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            # finish manuver if we reach the waypoint
            if(dist < 1.0):
                self.advanceWaypoint()

            if self.movement_mode == YAW_MODE:

                # point toward waypoint
                targetyaw = math.atan2(dy, dx)
                # rospy.logdebug("dx, dy: %f %f" % (dx, dy))
                # rospy.logdebug("Yaw, Target Yaw: %f %f" % (yaw, targetyaw))
                yaw_f64 = Float64()
                yaw_f64.data = targetyaw
                self.yaw_pub.publish(yaw_f64)

                # normalize change in yaw
                dyaw = yaw - targetyaw
                while(dyaw > math.pi):
                    dyaw = dyaw - 2*math.pi
                while(dyaw < -math.pi):
                    dyaw = dyaw + 2*math.pi

                # If we are pointing in about the right direction
                # start driving the distance to the target
                if(abs(math.degrees(dyaw)) < 7.5):
                    drive_f64 = Float64()
                    drive_f64.data = self.currentDrive + min(xy_dist, 3.0)
                    self.drive_pub.publish(drive_f64)
                    # rospy.logdebug(xy_dist)

            if self.movement_mode == STRAFE_MODE:

                targetyaw = math.radians(self.waypoint_yaw)

                # we are fixed yaw in this mode drive with drive/strafe
                yaw_f64 = Float64()
                yaw_f64.data = targetyaw
                self.yaw_pub.publish(yaw_f64)

                # normalize change in yaw
                dyaw = yaw - targetyaw
                while(dyaw > math.pi):
                    dyaw = dyaw - 2*math.pi
                while(dyaw < -math.pi):
                    dyaw = dyaw + 2*math.pi

                # If we are pointing in about the right direction
                # start driving the distance to the target
                if(abs(math.degrees(dyaw)) < 7.5):

                    drive_f64 = Float64()
                    drive_f64 = self.currentDrive + dx*math.cos(yaw) + dy*math.sin(yaw)
                    self.drive_pub.publish(drive_f64)

                    strafe_f64 = Float64()
                    strafe_f64 = self.currentStrafe + dx*math.sin(yaw) - dy*math.cos(yaw)
                    self.strafe_pub.publish(strafe_f64)

            # Set target depth
            depth_f64 = Float64()
            depth_f64.data = currentwaypoint.z
            self.depth_pub.publish(depth_f64)

    def driveStateCallback(self, data):
        self.currentDrive = data.data

    def strafeStateCallback(self, data):
        self.currentStrafe = data.data

    def run(self):

        self.namespace = rospy.get_param("~namespace")

        # setpoints to control sub motion
        self.depth_pub = rospy.Publisher(self.namespace + "/local_control/pid/depth/setpoint", Float64, queue_size=10)
        self.drive_pub = rospy.Publisher(self.namespace + "/local_control/pid/drive/setpoint", Float64, queue_size=10)
        self.strafe_pub = rospy.Publisher(self.namespace + "/local_control/pid/strafe/setpoint", Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher(self.namespace + "/local_control/pid/yaw/setpoint", Float64, queue_size=10)

        # get current accumulated strafe and drive to use for adjustments
        self.drive_state_sub = rospy.Subscriber(self.namespace + "/local_control/pid/drive/state", Float64, self.driveStateCallback)
        self.strafe_state_sub = rospy.Subscriber(self.namespace + "/local_control/pid/strafe/state", Float64, self.strafeStateCallback)

        # get current sub position to figure out how to get where we want
        self.pose_sub = rospy.Subscriber("/sensor_fusion/odometry/filtered", Odometry, self.odometryCallback)

        # service to add waypoints to drive to
        s = rospy.Service('addWaypoint', AddWaypoint, self.addWaypoint)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('WaypointNavigator')
    wn = WaypointNavigator()
    try:
        wn.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointNavigator Crashed")
