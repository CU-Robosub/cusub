#!/usr/bin/env python
"""
This class serves as the meta class for all tasks and objectives
"""

import smach
import rospy
from abc import ABCMeta, abstractmethod
from waypoint_navigator.msg import waypointAction, waypointGoal
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
import math
import actionlib
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty

from optparse import OptionParser
import inspect

# Waypoint Navigator Macros
YAW_MODE = 1
STRAFE_MODE = 2

class Task(smach.StateMachine):
    __metaclass__ = ABCMeta

    def __init__(self, outcomes):
        rospy.Subscriber('kill_sm', Empty, self.kill_sm)
        super(Task, self).__init__(outcomes=outcomes)

    @abstractmethod
    def initObjectives(self, searchAlg):
        """
        Initialize all objectives
        params:
        searchAlg : str, see search.py for examples
        """
        pass

    @abstractmethod
    def linkObjectives(self):
        """
        Link the objectives (initialized in initObjectives() to the state machine with the proper state transitions
        """
        pass

    def kill_sm(self, msg):
        # perhaps loop through every state and abort it, or provide a method that each execute first checks to see if we've been compromised & exits, when you CTRL + C
        cs = self.sm._current_state
        print cs
        self.sm._states[cs].request_abort()

    def priorList2Pose(self,list_xyz):
        pose = Pose()
        pose.position.x = list_xyz[0]
        pose.position.y = list_xyz[1]
        pose.position.z = list_xyz[2]
        return pose
"""
Objectives are subtasks within a task
They have:
     Pose of the sub (self.curPose)
     a waypointNavigator client (self.wayClient)
     Helper functions { goToPose(), getDistance() }
"""
class Objective(smach.State):

    pointReachedThreshold = 1.0

    def __init__(self, outcomes, objtv_name):
        assert type(objtv_name) == str
        self.name = objtv_name
        rospy.Subscriber('cusub_common/odometry/filtered', Odometry, self.sub_pose_cb)
        self._abort_requested = False
        self.wayClient = actionlib.SimpleActionClient('cusub_common/waypoint', waypointAction)
        self.started = False

        # initialize the pose
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        self.curPose = pose

        super(Objective, self).__init__(outcomes=outcomes)

    def goToPose(self, targetPose, useYaw=True):
        """
        Go to the point specified and block on success feedback from the waypoint client
        NOTE: if None is passed in for the targetPose, then the sub will wait where it currently is until being aborted
        Returns 0 if successfully reached the point
        Returns 1 if aborted
        """
        assert type(targetPose) == Pose or type(targetPose) == type(None)

        # Wait where we currently are until being aborted
        if type(targetPose) == type(None):
            rospy.logwarn("Objective has instructed Sub to wait where it is until being aborted")
            while not self.abort_requested():
                rospy.sleep(0.5)
            return "aborted"

        wpGoal = waypointGoal()
        wpGoal.goal_pose.pose.position = targetPose.position
        orientation_list = [ targetPose.orientation.x, \
                             targetPose.orientation.y, \
                             targetPose.orientation.z, \
                             targetPose.orientation.w ]
        (roll, pitch, targetYaw) = euler_from_quaternion(orientation_list)
        wpGoal.target_yaw = targetYaw
        if useYaw:
            wpGoal.movement_mode = YAW_MODE
        else:
            wpGoal.movement_mode = STRAFE_MODE

        self.wayClient.cancel_all_goals()
        rospy.sleep(0.2)
        self.wayClient.send_goal(wpGoal)
        rospy.loginfo("---goal sent to waypointNav")
        res = self.wayClient.get_result()

        while (res == None or not res.complete) and not rospy.is_shutdown():
            res = self.wayClient.get_result()

            if self.abort_requested():
                rospy.loginfo("---objective aborted, causing waypoint request to quit")
                return True

            rospy.sleep(0.25)

        rospy.loginfo("---reached pose")

        # if res != None:
        #     print("---wayClientResult: {}".format(res))
        #     print(type(res))

        return False

    def sub_pose_cb(self, msg):
        self.curPose = msg.pose.pose # store the pose part of the odom msg

    def getDistance(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z

        xy_dist = math.sqrt(dx**2 + dy**2)
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        return dist

    def abort_requested(self):
        return self._abort_requested

    def request_abort(self):
        rospy.loginfo("---requesting abort of " + self.name + " objective")
        self._abort_requested = True
    def clear_abort(self):
        self._abort_requested = False
