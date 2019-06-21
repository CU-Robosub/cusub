#!/usr/bin/env python
"""
These classes serves as the meta class for all tasks and objectives
The best way to write a new task is to learn by example from a simple task, such as start gate
"""

import smach
import rospy
from abc import ABCMeta, abstractmethod, abstractproperty
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

    @abstractproperty
    def name(self):
        pass

    @abstractmethod
    def initObjectives(self):
        """
        Initialize all objectives
        """
        pass

    @abstractmethod
    def linkObjectives(self):
        """
        Link the objectives to the state machine
        """
        pass

    def kill_sm(self, msg):
        # perhaps loop through every state and abort it, or provide a method that each execute first checks to see if we've been compromised & exits, when you CTRL + C
        cs = self.sm._current_state
        print cs
        self.sm._states[cs].request_abort()

    def getPrior(self):
        """
        Get the prior for the task from the rosparameter server

        Parameters
        ----------
        None

        Returns
        -------
        pose : Pose
             The prior pose of the task
        """

        list_xyz = rospy.get_param("tasks/" + self.name + "/prior")
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
        """ Objective initialization method

        Parameters
        ---------
        outcomes : list of strs
             strings of objective outcomes
        objtv_name : str
             name of the objective
        """
        self.name = objtv_name
        rospy.Subscriber('cusub_common/odometry/filtered', Odometry, self.sub_pose_cb)
        self._abort_requested = False
        self._replan_requested = False
        self.wayClient = actionlib.SimpleActionClient('/'+rospy.get_param('~robotname')+'/cusub_common/waypoint', waypointAction)
        self.started = False

        # initialize the pose
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        self.curPose = pose

        super(Objective, self).__init__(outcomes=outcomes)

    def goToPose(self, targetPose, useYaw=True):
        """ Traverse to the targetPose given

        Parameters
        ----------
        targetPose : Pose
             The pose to navigate to
             If None, the sub will stop where it currently is and wait to be aborted
        useYaw : bool
             true : the waypoint navigtator will use yaw mode to navigate to the target pose
             false : use strafe-drive mode to the target pose

        Returns
        -------
        bool : success/aborted
             1 aborted
             0 success, waypoint reached
        """
        if type(targetPose) == type(None):  # Wait where we currently are until being aborted
            rospy.logwarn("Objective has instructed Sub to wait where it is until being aborted")
            while not self.abort_requested():
                rospy.sleep(0.5)
            return "aborted"

        wpGoal = waypointGoal()
        if type(targetPose) == PoseStamped:
            wpGoal.goal_pose.pose.position = targetPose.pose.position
            wpGoal.goal_pose.pose.orientation = targetPose.pose.orientation
            wpGoal.goal_pose.header.frame_id = targetPose.header.frame_id
        else:
            wpGoal.goal_pose.pose.position = targetPose.position
            wpGoal.goal_pose.pose.orientation = targetPose.orientation
            wpGoal.goal_pose.header.frame_id = 'leviathan/description/odom'

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
        return False

    def sub_pose_cb(self, msg):
        self.curPose = msg.pose.pose # store the pose part of the odom msg

    def getDistance(self, point1, point2):
        """ Get distance between 2 points """
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

    def replan_requested(self):
        return self._abort_requested
    def request_replan(self):
        rospy.loginfo("---requesting replan of " + self.name + " objective")
        self._replan_requested = True
    def clear_replan(self):
        self._replan_requested = False
