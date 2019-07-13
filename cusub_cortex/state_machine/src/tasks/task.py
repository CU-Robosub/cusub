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
from darknet_multiplexer.srv import DarknetCameras
import numpy as np
import tf

from optparse import OptionParser
import inspect

POSE_REACHED_THRESHOLD = 0.3

# Waypoint Navigator Macros
YAW_MODE = 1
STRAFE_MODE = 2
BACKUP_MODE = 3

class Task(smach.StateMachine):
    __metaclass__ = ABCMeta

    outcome = ["manager"]

    def __init__(self):
        rospy.Subscriber('kill_sm', Empty, self.kill_sm)
        super(Task, self).__init__(
            outcomes=self.outcome,\
            input_keys=['timeout_obj'],\
            output_keys=['timeout_obj', 'outcome'])

    @abstractproperty
    def name(self):
        pass

    @abstractmethod
    def init_objectives(self):
        """
        Initialize all objectives
        """
        pass

    @abstractmethod
    def link_objectives(self):
        """
        Link the objectives to the state machine
        """
        pass

    def kill_sm(self, msg):
        # perhaps loop through every state and abort it, or provide a method that each execute first checks to see if we've been compromised & exits, when you CTRL + C
        cs = self.sm._current_state
        print cs
        self.sm._states[cs].request_abort()

    def get_prior_topic(self):
        """
        Get the prior for the task from the rosparameter server

        Parameters
        ----------
        None

        Returns
        -------
        str : String
             The rosparam name of prior
        """
        return "tasks/" + self.name + "/prior"

"""
Objectives are subtasks within a task
They have:
     Pose of the sub (self.cur_pose)
     a waypointNavigator client (self.wayClient)
     Helper functions { go_to_pose(), get_distance() }
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
        self.using_darknet = rospy.get_param("~using_darknet")

        # initialize the pose
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        self.cur_pose = pose
        self.cancel_goal = False

        super(Objective, self).__init__(
            outcomes=outcomes,\
            input_keys=['timeout_obj'],\
            output_keys=['timeout_obj', 'outcome'])

    def configure_darknet_cameras(self, camera_bool_list):
        """
        Configures which cameras for darknet to use.

        Params
        ------
        camera_bool_list : list of bools, length 6
            Darknet active cameras, 1 for use
            [ occam0, 1, 2, 3, 4, downcam ]

        Returns
        -------
        bool
            1 success
            0 failed
        """
        if not self.using_darknet:
            return False
        rospy.wait_for_service("cusub_perception/darknet_multiplexer/configure_active_cameras")
        try:
            darknet_config = rospy.ServiceProxy("cusub_perception/darknet_multiplexer/configure_active_cameras", DarknetCameras)
            resp1 = darknet_config(camera_bool_list)
            return True
        except rospy.ServiceException, e:
            rospy.logerr("Darknet Camera Config Service call failed: %s"%e)
            return False

    def go_to_pose(self, target_pose, move_mode="yaw"):
        """ Traverse to the target_pose given

        Parameters
        ----------
        target_pose : Pose
             The pose to navigate to
             If None, the sub will stop where it currently is and wait to be aborted
        move_mode : str
             "yaw" : the waypoint navigtator will use yaw mode to navigate to the target pose
             "strafe" : use strafe-drive mode to the target pose
             "backup" : turn 180 deg away from object, backup to target point, turn to target orientation

        Returns
        -------
        bool : success/aborted
             1 aborted
             0 success, waypoint reached
        """
        if type(target_pose) == type(None):  # Wait where we currently are until being aborted
            rospy.logwarn("Objective has instructed Sub to wait where it is until being aborted")
            while not self.abort_requested():
                rospy.sleep(0.5)
            return "aborted"

        self.go_to_pose_non_blocking(target_pose, move_mode)
        return self.block_on_reaching_pose(target_pose)

    def go_to_pose_non_blocking(self, target_pose, move_mode="yaw"):
        wpGoal = waypointGoal()
        wpGoal.goal_pose.pose.position = target_pose.position
        wpGoal.goal_pose.pose.orientation = target_pose.orientation
        wpGoal.goal_pose.header.frame_id = 'leviathan/description/odom'

        if move_mode == "backup":
            wpGoal.movement_mode = BACKUP_MODE
        elif move_mode == "strafe":
            wpGoal.movement_mode = STRAFE_MODE
        else:
            wpGoal.movement_mode = YAW_MODE

        self.wayClient.cancel_all_goals()
        rospy.sleep(0.2)
        self.wayClient.send_goal(wpGoal)
        rospy.loginfo("---goal sent to waypointNav")
    
    def cancel_wp_goal(self):
        """ An alternative to abort for cancelling the current pose goal that prints a lot less """
        self.cancel_goal = True
    
    def block_on_reaching_pose(self, target_pose):
        res = self.wayClient.get_result()
        while (res == None or not res.complete) and not rospy.is_shutdown():
            res = self.wayClient.get_result()

            if self.abort_requested():
                self.wayClient.cancel_goal()
                rospy.loginfo("---waypoint quitting")
                return True
            elif self.get_distance(self.cur_pose.position, target_pose.position) < POSE_REACHED_THRESHOLD:
                self.wayClient.cancel_goal()
                return False
            elif self.cancel_goal:
                self.wayClient.cancel_goal()
                self.cancel_goal = False
                return True
            rospy.sleep(0.25)
        rospy.loginfo("---reached pose")
        return False

    def sub_pose_cb(self, msg):
        self.cur_pose = msg.pose.pose # store the pose part of the odom msg

    def get_distance_xy(self, point1, point2):
        """ Get xy distance between 2 points """
        dx = point2.x - point1.x
        dy = point2.y - point1.y

        xy_dist = math.sqrt(dx**2 + dy**2)
        return xy_dist


    def get_distance(self, point1, point2):
        """ Get distance between 2 points """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z

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

    @staticmethod
    def get_pose_between(cur_pose, object_pose, dist_from_object):
        """
        @brief Calculates the pose between the sub and object with dist_from_object

        xy line drawn from cur_pose to object_pose, z is pulled from object_pose

        Parameters
        ----------
        cur_pose : Pose
            pose of the sub
        object_pose : Pose
            pose of the object
        dist_from_object : float
            x meters away from the object on an xy line to the sub,
            z is pulled from object
        Returns
        -------
        Pose
            pose between object and sub
        """
        # Find line from sub to buoy
        if ( round(cur_pose.position.x, 2) == round(object_pose.position.x,2) ): # Avoid infinite slope in the polyfit
            object_pose.position.x += 0.1
        if ( round(cur_pose.position.y, 2) == round(object_pose.position.y,2) ): # Avoid infinite slope in the polyfit
            object_pose.position.y -= 0.1

        x_new = object_pose.position.x
        y_new = object_pose.position.y

        # Adjust buoy pose behind the buoy
        x2 = np.array([cur_pose.position.x, x_new])
        y2 = np.array([cur_pose.position.y, y_new])
        m2, b2 = np.polyfit(x2,y2,1)
        m2 = round(m2, 2)
        b2 = round(b2, 2)
        x_hat2 = np.sqrt( ( dist_from_object**2) / (m2**2 + 1) )
        if x_new > cur_pose.position.x:
            x_hat2 = -x_hat2
        x_new2 = x_new + x_hat2
        y_new2 = x_new2 * m2 + b2

        # Find target yaw
        dx = object_pose.position.x - x_new2
        dy = object_pose.position.y - y_new2
        target_yaw = np.arctan2(dy, dx)
        quat_list = tf.transformations.quaternion_from_euler(0,0, target_yaw)

        # Make Pose Msg
        target_pose = Pose()
        target_pose.position.x = x_new2
        target_pose.position.y = y_new2
        target_pose.position.z = object_pose.position.z
        target_pose.orientation.x = quat_list[0]
        target_pose.orientation.y = quat_list[1]
        target_pose.orientation.z = quat_list[2]
        target_pose.orientation.w = quat_list[3]

        return target_pose

    @staticmethod
    def get_pose_behind(cur_pose, object_pose, dist_behind_object):
        """
        @brief Gets the pose from the cur_pose to object_pose with dist_behind_object

        xy line drawn from cur_pose to object_pose, z is pulled from object_pose
        Uses cur_pose's orientation

        Intended for slaying bouys

        Parameters
        ----------
        cur_pose : Pose
            pose of the sub
        object_pose : Pose
            pose of the object
        dist_behind_object : float
            distance behind object, 

        Returns
        -------
        Pose
            pose behind the object
        
        """
        # Find line from sub to buoy
        if ( round(cur_pose.position.x, 2) == round(object_pose.position.x,2) ): # Avoid infinite slope in the polyfit
            object_pose.position.x += 0.1
        if ( round(cur_pose.position.y, 2) == round(object_pose.position.y,2) ): # Avoid infinite slope in the polyfit
            object_pose.position.y -= 0.1

        x2 = np.array([cur_pose.position.x, object_pose.position.x])
        y2 = np.array([cur_pose.position.y, object_pose.position.y])
        m2, b2 = np.polyfit(x2,y2,1)
        m2 = round(m2, 2)
        b2 = round(b2, 2)
        x_hat2 = np.sqrt( ( dist_behind_object**2) / (m2**2 + 1) )
        if object_pose.position.x <= cur_pose.position.x:
            x_hat2 = -x_hat2
        x_new2 = object_pose.position.x + x_hat2
        y_new2 = x_new2 * m2 + b2

        target_pose = Pose()
        target_pose.position.x = x_new2
        target_pose.position.y = y_new2
        target_pose.position.z = cur_pose.position.z
        target_pose.orientation = cur_pose.orientation
        return target_pose

class Timeout():
    """
    @brief Timeout object for tasks
    """
    timer = None

    def set_new_time(self, seconds):
        if self.timer != None:
            self.timer.shutdown()
        self.timed_out = False
        self.timer = rospy.Timer(rospy.Duration(seconds), self.timer_callback)

    def timer_callback(self, msg):
        self.timer.shutdown()
        rospy.logerr("Task Timed Out")
        self.timed_out = True

    def timed_out(self):
        return self.timed_out