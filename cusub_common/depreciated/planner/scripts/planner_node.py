#!/usr/bin/env python

"""
Planner Node

- Input: goal pose from the Global_Controller/state_machine_node, 
* converts the msg into local_controller frame
* Sets the setpoints for the pid loops in the controller pkg
- Recursively corrects for drift error in the local controller
* Publishes new correction points to eventually reach requested x,y,z,w from state_machine_node

FRAMES
- Global : outputted from the ekf inside the sensor_fusion package
    12 DOF msg (geometry_twist), 
- Controller : Drive,strafe,depth,heading,roll(always zero),pitch (always zero)
    The controller accumulates how far it has gone in drive (its forward direction), how much it has strafed (sideways direction) and how much it has turned and dove. 
    The planner node picks out a planned movement converts the 12 DOF from state_machine_node into its equivalent drive,strafe,depth and heading. It publishes this route by changing one 6DOF element at a time in the local planner.

2 problems, first what's with the short depth
second, why are we turning a strange way, it's something with going to one coord, let's test them all
"""

# launch file = bounding radius on the xyzw point we want to be at

__author__ = "luke barbier"
__copyright__ = "Copyright 2018, CU RoboSub"
__license__ = "MIT"
__version__ = "1.1"
__email__ = "luba6098@colorado.edu"
__status__ = "Development"

import rospy
import tf
import numpy as np
import sys
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

# to simplify the pid interface (no 20 callbacks in this file...)
from planner.controller_if import Controller_If # check if this correctly imports
from planner.simplePlanner import *

class Planner_Node():
    
    def __init__(self):
        
        self.globalPoseEuler = EulerPose(0,0,0,0)
        
        # load params from launch file
        self.strafeThreshDist = float(rospy.get_param('~strafeThreshDist', 1))
        self.strafeThreshAngle = float(rospy.get_param('~strafeThreshAngle', np.pi/4))
        self.controlEffortThresh = float(rospy.get_param('~controlEffortThresh', 4))
        self.goalRadiusThresh = float(rospy.get_param('~goalRadiusThresh', 0.5))
        self.goalAngleThresh = float(rospy.get_param('~goalAngleThresh', np.pi/8))

        self.depthGoal = 0.3
        self.driveGoal = 0
        self.strafeGoal = 0
        self.yawGoal = 0
        self.pitchGoal = 0
        self.rollGoal = 0

        # load this as a param soon
        reachedGoalTopic = '/reached_goal'
        self.reachedPub = rospy.Publisher(reachedGoalTopic, Empty, queue_size=10) # we'll tell the state machine that we reached it's goal

        poseTopic = '/sensor_fusion/odometry/filtered' # should be geometryTwist msg
        goalPoseTopic = rospy.get_param('~goal_pose_topic', '/local_control/planner_goal')
        rospy.loginfo("goal pose topic= " + goalPoseTopic)
        rospy.Subscriber(poseTopic, Odometry, self.poseCallback)
        rospy.Subscriber(goalPoseTopic, Pose, self.goalPoseCallback)

        self.contIf = Controller_If()
        rospy.loginfo("Planner Node finished initializing")
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timerPubSetpoints)

    def goalPoseCallback(self, pose): # takes in geometry_msgs/Pose goals
        rospy.loginfo("Goal Pose Received")
#        rospy.sleep(2)
        goal_x = pose.position.x
        goal_y = pose.position.y
        goal_z = pose.position.z
        orientation = pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])
        goal_w = yaw

        # loop in the approach of our goal point until we've reached it 
        while not rospy.is_shutdown(): 
            
            goalPoseEuler = EulerPose(goal_x, goal_y, goal_z, goal_w)
        
            # determine if we can strafe (calculate strafe movement)
            # code logic to determine if we can strafe

            # we won't strafe here, get the relative movement
            relMovement = self.getRelMovementNoStrafe(goalPoseEuler)
            rospy.loginfo("Current Pose: (" + str(self.globalPoseEuler.x) + ", " + str(self.globalPoseEuler.y) + ")")
            # we've been given a relative movement let's transform to its global meaning

            # uncomment before running
            globMovement = self.relative2Global( relMovement )
            rospy.loginfo("relMovement: " + str(relMovement))
            self.executeMovement( globMovement )
            #rospy.loginfo("sleeping") ; rospy.sleep(10)
        
            if self.checkReached( goalPoseEuler ): # check if we've reached our destination
                break
            else:
                rospy.logwarn("Haven't reached goal, recursively looping")
                rospy.logerr("Pose: (" + str(self.globalPoseEuler.x) + ", " + str(self.globalPoseEuler.y) + ")")

        # indicate that we've reached our point
        rospy.loginfo("We've reached our target goal pose")
        self.reachedPub.publish( Empty() )
        rospy.logerr("Pose: (" + str(self.globalPoseEuler.x) + ", " + str(self.globalPoseEuler.y) + ")")

    def relative2Global(self, mov):
        rospy.loginfo(str(mov))
        globMov = []

        # depth
        globMov.append( mov[0] )

        # yaw
        globMov.append( mov[1] ) # we just stick with this headingx

        # strafe
        if mov[2] is not None:            
            globMov.append( mov[2] + self.contIf.strafe['state'] )
        else:
            globMov.append(None)
            
        # drive
        if mov[3] is not None:
            globMov.append( mov[3] + self.contIf.drive['state'] )
        else:
            globMov.append(None)
            
        # yaw
        globMov.append( mov[4] ) # final heading here
        
        return globMov

    def checkReached(self, goalPoseEuler):
        if not isinstance(goalPoseEuler, EulerPose):
            rospy.logerr("Incorrent arguments to checkReached, provide an EulerPose")
            return False

        # let's determine a radius from where we want to be 
        diff_x = self.globalPoseEuler.x - goalPoseEuler.x
        diff_y = self.globalPoseEuler.y - goalPoseEuler.y
        diff_z = self.globalPoseEuler.z - goalPoseEuler.z

        rad = np.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

        # check if we've reached our pose
        if rad < self.goalRadiusThresh:
            return True
        else:
            return False
        
    def timerPubSetpoints(self, msg):
        self.contIf.pubDepthGoal( self.depthGoal )
        self.contIf.pubYawGoal( self.yawGoal )
        self.contIf.pubStrafeGoal( self.strafeGoal )
        self.contIf.pubDriveGoal( self.driveGoal )
        self.contIf.pubPitchGoal( self.pitchGoal )
        self.contIf.pubRollGoal( self.rollGoal )
        
    def executeMovement(self, mov): # all movements in the form of [depth, turn, strafe, drive, turn] (to skip an aspect pass in None)
        """
        @param
        movement : list of floats, to skip a movement pass in None
        """

        # ONLY CHANGE ONE DIMENSION AT A TIME IN THE MOVEMENT
        
        # depth
        count = 0
        wait = 100
        
        print(mov)
        if not self.contIf.depth['controlEffort'] or \
           not self.contIf.drive['controlEffort'] or \
           not self.contIf.strafe['controlEffort'] or \
           not self.contIf.yaw['controlEffort'] :
            rospy.logerr("control effort not publishing!")
            sys.exit(1)
            
        # depth
        if mov[0]:
            self.depthGoal = mov[0]
            rospy.sleep(0.5) # let the motors respond
            rospy.loginfo("attemping depth")
            # block on the movement not being completed
            while count < wait:
                #self.contIf.pubDepthGoal( mov[0] )
                if abs(self.contIf.depth['controlEffort']) < 25:
                    count += 1
                else:
                    count = 0
            rospy.loginfo("Finish Depth Goal")
        else:
            rospy.logwarn("No depth movement")
        count = 0
        # turn
        if mov[1]:
            self.yawGoal = mov[1]
            rospy.sleep(0.5)
            rospy.loginfo("attemping turn")
            while count < wait:
#                self.contIf.pubYawGoal( mov[1] )
                if abs(self.contIf.yaw['controlEffort']) < 15:
                    count += 1
                else:
                    count = 0
            rospy.loginfo("Finish Turn Goal")
        else:
            rospy.logwarn("No turn movement")
        count = 0
        # strafe
        if mov[2]:
            self.strafeGoal = mov[2]
            rospy.sleep(0.5)
            rospy.loginfo("attemping strafe")
            while count < wait:
#                self.contIf.pubStrafeGoal( mov[2] )
                if abs(self.contIf.strafe['controlEffort']) < 15:
                    count += 1
                else:
                    count = 0
            rospy.loginfo("Finish Strafe Goal")
        else:
            rospy.logwarn("No strafe movement")
        count = 0
        # drive
        if mov[3]:
            self.driveGoal = mov[3]
            rospy.sleep(0.5)
            rospy.loginfo("attemping drive")
            while count < wait:
#                self.contIf.pubDriveGoal( mov[3] )
                if abs(self.contIf.drive['controlEffort']) < 15:
                    count += 1
                else:
                    count = 0
            rospy.loginfo("Finish Drive Goal")
        else:
            rospy.logwarn("No drive movement")
        count = 0
        # turn
        if mov[4]:
            self.yawGoal = mov[4]
            rospy.sleep(0.5)
            rospy.loginfo("attemping turn")
            while count < wait:
#                self.contIf.pubYawGoal( mov[4] )
                if abs(self.contIf.yaw['controlEffort']) < 15:
                    count += 1
                else:
                    count = 0            
            rospy.loginfo("Finish Turn2 Goal")
        else:
            rospy.logwarn("No turn2 movement")
        count = 0
        
    def poseCallback(self, odom): 
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])
        w = yaw

        self.globalPoseEuler = EulerPose(x,y,z,w)

    def getRelMovementNoStrafe(self, goalPose):
        # NOTE THE DEGREES ARE NOT A CHANGE BUT THE ACTUAL DEGREES THE SUB SHOULD TURN TOWARD (whereas drive and strafe are differences from the current state ie drive should increase by __ and strafe should increase by __)

        # this is z, theta, strafe, drive, theta
        
        if not isinstance(goalPose, EulerPose):
            rospy.logerr("Incorrent arguments, provide an EulerPose")
            return
        
        diff_x = goalPose.x - self.globalPoseEuler.x
        diff_y = goalPose.y - self.globalPoseEuler.y
#        diff_z = goalPose.z - self.globalPoseEuler.z

        movement=[]
        movement.append(goalPose.z) # let's put the global goal since it's the same

        h = np.sqrt( diff_x**2 + diff_y**2 )
        
        if h == 0:
            print("Already @ the point in x and y")
            movement.append(goalPose.w)
            movement.append(None)
            movement.append(None)
            movement.append(None)
            return movement

        theta = np.arccos( diff_x / h )
        if diff_y < 0:
            theta = -theta

#        movement.append(theta)
        movement.append(theta)

        # add in zero strafe
        movement.append(None)

        # append the dist
        movement.append(h)

        # append final orientation
        movement.append(goalPose.w)

        return movement
    

if __name__ == "__main__":
    rospy.init_node('path_planner_node')
    p = Planner_Node()
    rospy.spin()
