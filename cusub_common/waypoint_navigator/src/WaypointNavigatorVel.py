#!/usr/bin/python2

import rospy
import tf
import math

from enum import Enum

from waypoint_navigator.srv import *

from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from cusub_print.cuprint import CUPrint, bcolors
import numpy as np

YAW_MODE = 1
STRAFE_MODE = 2
BACKUP_MODE = 3

REACHED_THRESHOLD = 0.25
MAX_VEL = 1.0

class WaypointNavigator(object):

    def __init__(self):
        self.cuprint = CUPrint("Waypoint Nav")
        self.waypointlist = []

        self.waypoint = None

        self.movement_mode = STRAFE_MODE
        self.max_vel_threshold = 3
        self.waypoint_yaw = 0

    def addWaypoint(self, req):

        self.cuprint("received waypoint req")

        self.waypointlist.append(req.waypoint)
        if(self.waypoint is None):
            self.waypoint = self.waypointlist[0]

        return []
    
    def toggleControl(self, req):
        self.cuprint("toggling control: " + str(req.waypoint_controlling))
        self.controlling_pids = req.waypoint_controlling
        if(req.waypoint_controlling):
            self.waypoint = None
            self.do_freeze()
        return True

    def publish_controlling_pids(self, msg):
        """
        Publish whether the waypoint nav is currently controlling the pid loops
        """
        msg = Bool()
        msg.data = self.controlling_pids
        self.control_pub.publish(msg)

    def do_freeze(self):
        self.yawFreeze = self.currentYaw
        self.driveFreeze = self.currentDrive
        self.strafeFreeze = self.currentStrafe
        self.depthFreeze = self.currentDepth
        self.freeze_controls()

    def freeze_controls(self):

        if self.yawFreeze == None:
            return

        # YAW
        yaw_f64 = Float64()
        yaw_f64.data = self.yawFreeze
        self.yaw_pub.publish(yaw_f64)

        # STRAFE
        msg = Float64()
        msg.data = self.strafeFreeze
        self.strafe_pub.publish(msg)

        # DRIVE
        msg = Float64()
        msg.data = 0.0
        self.drive_pub.publish(msg)

        # DEPTH
        msg = Float64()
        msg.data = 0.0
        self.depth_pub.publish(msg)

    def advance_waypoint(self):
        self.cuprint("waypoint reached!")
        if(len(self.waypointlist) > 0):
            self.waypoint = self.waypointlist[0]

    def check_done(self, dist):
        # finish manuver if we reach the waypoint and are facing the correct direction
        if(dist < REACHED_THRESHOLD):
            self.advance_waypoint()
        else:
            self.cuprint("distance: " + bcolors.HEADER + str(dist) + bcolors.ENDC, print_prev_line=True)

    def odometryCallback(self, odom):

        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])

        if(self.waypoint is not None and self.controlling_pids):

            currentwaypoint = self.waypoint

            dx = currentwaypoint.x - position.x
            dy = currentwaypoint.y - position.y
            dz = currentwaypoint.z - position.z

            xy_dist = math.sqrt(dx**2 + dy**2)
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

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
                    if xy_dist > self.max_vel_threshold:
                        drive_f64 = MAX_VEL
                    else:
                        drive_f64 = MAX_VEL * (xy_dist / self.max_vel_threshold)

                    # drive_f64.data = self.currentDrive + min(xy_dist, self.carrot_drive)
                    self.drive_pub.publish(drive_f64)
                    # rospy.logdebug(xy_dist)

                    self.check_done(dist)
                else:
                    self.drive_pub.publish(0.0)

            elif self.movement_mode == STRAFE_MODE:

                targetyaw = self.waypoint_yaw

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
                    drive_delta = dx*math.cos(yaw) + dy*math.sin(yaw)
                    
                    if drive_delta > self.max_vel_threshold:
                        drive_f64 =  MAX_VEL
                    else:
                        drive_f64 =  MAX_VEL * (drive_delta / self.max_vel_threshold)

                    # drive_f64 = self.currentDrive + dx*math.cos(yaw) + dy*math.sin(yaw)

                    strafe_f64 = Float64()
                    strafe_delta = dx*math.sin(yaw) - dy*math.cos(yaw)
                    if strafe_delta > self.max_vel_threshold:
                        strafe_f64 = - MAX_VEL
                    else:
                        strafe_f64 = - MAX_VEL * (strafe_delta / self.max_vel_threshold)
                    # strafe_f64 = self.currentStrafe + dx*math.sin(yaw) - dy*math.cos(yaw)
                    tot_vel = np.linalg.norm([drive_f64, strafe_f64])
                    if tot_vel > MAX_VEL:
                        drive_f64 /= tot_vel
                        strafe_f64 /= tot_vel


                    self.drive_pub.publish(drive_f64)
                    self.strafe_pub.publish(strafe_f64)

                    # finish manuver if we reach the waypoint and are facing the correct direction
                    self.check_done(dist)
                        
            elif self.movement_mode == BACKUP_MODE:

                # point toward waypoint
                targetyaw = math.atan2(dy, dx) + math.pi
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
                    if xy_dist > self.max_vel_threshold:
                        drive_f64 = - MAX_VEL
                    else:
                        drive_f64 = - MAX_VEL * (xy_dist / self.max_vel_threshold)
                    # drive_f64.data = self.currentDrive - min(xy_dist, 3.0)
                    self.drive_pub.publish(drive_f64)
                    # rospy.logdebug(xy_dist)

                    self.check_done(dist)

            # Set target depth
            depth_f64 = Float64()
            depth_f64.data = currentwaypoint.z
            self.depth_pub.publish(depth_f64)

        elif self.controlling_pids:
            self.freeze_controls()

    def driveStateCallback(self, data):
        self.currentDrive = data.data

    def strafeStateCallback(self, data):
        self.currentStrafe = data.data

    def yawStateCallback(self, data):
        self.currentYaw = data.data

    def depthStateCallback(self, data):
        self.currentDepth = data.data

    def run(self):
	
        self.carrot_drive = rospy.get_param("waypoint_navigator/carrot_drive", 3.0)

        self.yawFreeze = None
        self.driveFreeze = None
        self.strafeFreeze = None
        self.depthFreeze = None

        # setpoints to control sub motion
        self.depth_pub = rospy.Publisher("motor_controllers/pid/depth/setpoint", Float64, queue_size=10)
        self.drive_pub = rospy.Publisher("motor_controllers/pid/drive_vel/setpoint", Float64, queue_size=10)
        self.strafe_pub = rospy.Publisher("motor_controllers/pid/strafe_vel/setpoint", Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher("motor_controllers/pid/yaw/setpoint", Float64, queue_size=10)

        # get current accumulated strafe and drive to use for adjustments. (Depth and Yaw just used for freezing the sub)
        self.drive_state_sub = rospy.Subscriber("motor_controllers/pid/drive_vel/state", Float64, self.driveStateCallback)
        self.strafe_state_sub = rospy.Subscriber("motor_controllers/pid/strafe_vel/state", Float64, self.strafeStateCallback)
        self.yaw_state_sub = rospy.Subscriber("motor_controllers/pid/yaw/state", Float64, self.yawStateCallback)
        self.depth_state_sub = rospy.Subscriber("motor_controllers/pid/depth/state", Float64, self.depthStateCallback)

        self.cuprint("waiting for drive state")
        rospy.wait_for_message("motor_controllers/pid/drive_vel/state", Float64)
        self.cuprint("waiting for depth state")
        rospy.wait_for_message("motor_controllers/pid/depth/state", Float64)
        self.cuprint("waiting for yaw state")
        rospy.wait_for_message("motor_controllers/pid/yaw/state", Float64)
        self.cuprint("waiting for strafe state")
        rospy.wait_for_message("motor_controllers/pid/strafe_vel/state", Float64)

        # get current sub position to figure out how to get where we want
        self.pose_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odometryCallback)

        # service to add waypoints to drive to
        s = rospy.Service('addWaypoint', AddWaypoint, self.addWaypoint)

        self.controlling_pids = True

        self.cuprint("opening toggle service")

        s2 = rospy.Service('toggleWaypointControl', ToggleControl, self.toggleControl)

        self.control_pub = rospy.Publisher("waypoint_controlling_pids", Bool, queue_size=10)
        control_pub_timer = rospy.Timer(rospy.Duration(0.1), self.publish_controlling_pids)

        rospy.spin()

if __name__ == '__main__':
    print ("starting Toggle node")	
    rospy.init_node('WaypointNavigator')
    wn = WaypointNavigator()
    try:
        wn.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointNavigator Crashed")
