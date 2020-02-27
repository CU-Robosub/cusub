#!/usr/bin/env python
"""
Meta Classes for all Tasks and Objectives.
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
from state_machine.msg import TaskStatus
from waypoint_navigator.srv import *
from cusub_print.cuprint import CUPrint

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

    def __init__(self, name):
        self.cuprint = CUPrint(name)
        self.name = name
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

    def get_prior_param(self):
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
        return "tasks/" + self.name.lower() + "/prior"

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
        rospy.Subscriber('cusub_cortex/state_machine/task_status', TaskStatus, self.current_tasks_cb)
        self.task_dict = {}
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

        self.cuprint = CUPrint(objtv_name)

        super(Objective, self).__init__(
            outcomes=outcomes,\
            input_keys=['timeout_obj'],\
            output_keys=['timeout_obj', 'outcome'])

    def toggle_waypoint_control(self, sm_take_control):
        wayToggle = rospy.ServiceProxy('cusub_common/toggleWaypointControl', ToggleControl)
        try:
            res = wayToggle(not sm_take_control)
            return True
        except rospy.ServiceException, e:
            self.cuprint("Toggling waypoint nav control failed: " + str(e), warn=True)
            return False

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
        self.cuprint("configuring darknet cameras")
        if not self.using_darknet:
            return False
        rospy.wait_for_service("cusub_perception/darknet_multiplexer/configure_active_cameras")
        try:
            darknet_config = rospy.ServiceProxy("cusub_perception/darknet_multiplexer/configure_active_cameras", DarknetCameras)
            resp1 = darknet_config(camera_bool_list)
            self.cuprint("...configured")
            return True
        except rospy.ServiceException, e:
            rospy.logerr("Darknet Camera Config Service call failed: %s"%e)
            return False

    def go_to_pose(self, target_pose, timeout_obj, replan_enabled=True, move_mode="yaw"):
        """ @brief traverses to the target_pose given, blocks until reached

        Parameters
        ----------
        target_pose : Pose
             The pose for the sub to navigate to
             Passed by copy instead of reference (like self.target_pose) b/c replanning now allowed
        timeout_obj : almost certainly this is userdata.timeout_obj
        replan_enabled : bool
            True for allowing replans to interrupt the mission
            False for preventing replans from happening
        move_mode : str
             "yaw" : the waypoint navigtator will use yaw mode to navigate to the target pose
             "strafe" : use strafe-drive mode to the target pose
             "backup" : turn 180 deg away from object, backup to target point, turn to target orientation

        Returns
        -------
        bool : 
            0 success, waypoint reached
            1 timedout or replan_requested, user must check the value of 
                timeout_obj.timed_out when this result is returned to determine cause of failure
        """
        self.go_to_pose_non_blocking(target_pose, move_mode)
        return self.block_on_reaching_pose(target_pose, timeout_obj, replan_enabled)

    def go_to_pose_non_blocking(self, target_pose, move_mode="yaw", log_print=True):
        """
        @brief sends a pose to waypoint navigator and returns
        """
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
        if log_print:
            self.cuprint("goal sent to waypointNav")
    
    def block_on_reaching_pose(self, target_pose, timeout_obj, replan_enabled=True):
        """
        @brief blocks on the already sent waypoint goal

        Parameters
        ----------
        target_pose : Pose
             The pose for the sub to navigate to
        timeout_obj : almost certainly this is userdata.timeout_obj

        Returns
        -------
        bool : 
            0 success, waypoint reached
            1 timedout or replan_requested, user must check the value of 
                timeout_obj.timed_out when this result is returned to determine cause of failure
        """
        self.clear_replan()
        res = self.wayClient.get_result()
        while (res == None or not res.complete) and not rospy.is_shutdown():
            res = self.wayClient.get_result()
            if self.check_reached_pose(target_pose):
                self.cancel_way_client_goal()
                return False
            elif timeout_obj.timed_out:
                self.cancel_way_client_goal()
                return True
            elif self.replan_requested() and replan_enabled:
                self.cancel_way_client_goal()
                return True
            rospy.sleep(0.25)
        return False

    def cancel_way_client_goal(self):
        self.wayClient.cancel_goal()

    def check_reached_pose(self, target_pose, threshold=POSE_REACHED_THRESHOLD):
        return self.get_distance(self.cur_pose.position, target_pose.position) < threshold

    def sub_pose_cb(self, msg):
        self.cur_pose = msg.pose.pose # store the pose part of the odom msg

    def get_distance_xy(self, point1, point2):
        """ Get xy distance between 2 points """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        return math.sqrt(dx**2 + dy**2)

    def get_distance(self, point1, point2):
        """ Get distance between 2 points """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def replan_requested(self):
        return self._replan_requested
    def request_replan(self):
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

    """
    callback to update our prior dict
    """
    def current_tasks_cb(self, msg):
        for i in range(0, len(msg.task_statuses), 2):
            task = msg.task_statuses[i]
            status = msg.task_statuses[i+1]
            
            self.task_dict[task] = status

        
    """
    Will loop through the next tasks and update the relative priors
    """
    def update_next_priors(self, current_task):
        xyzframe_list = rospy.get_param("tasks/"+current_task+"/prior")
        cur_estimated_prior = Point()
        cur_estimated_prior.x = xyzframe_list[0]
        cur_estimated_prior.y = xyzframe_list[1]

        for task, status in self.task_dict.iteritems():
            if status == "queued": 
                task_prior_name = "tasks/"+task+"/prior"

                xyzframe_list = rospy.get_param(task_prior_name)
                task_prior_pt = Point()
                task_prior_pt.x = xyzframe_list[0]
                task_prior_pt.y = xyzframe_list[1]
                task_prior_pt.z = xyzframe_list[2]
                
                delta_x = task_prior_pt.x - cur_estimated_prior.x
                delta_y = task_prior_pt.y - cur_estimated_prior.y

                adjusted_prior = [self.cur_pose.position.x + delta_x, \
                                  self.cur_pose.position.y + delta_y, \
                                  task_prior_pt.z]
                self.cuprint("updating prior for " + task)
                rospy.set_param(task_prior_name, adjusted_prior)
                
class Timeout():
    """
    @brief Timeout object for tasks
    """
    timer = None

    def __init__(self, name=""):
        if name == "":
            self.cuprint = CUPrint("SM Timeout Object")
        else:
            self.cuprint = CUPrint(name + "/Timeout Object")

    def set_new_time(self, seconds, print_new_time=True):
        """ In objectives reference like: userdata.timeout_obj.set_new_time(4) """
        if self.timer != None:
            self.timer.shutdown()
        self.timed_out = False
        if seconds != 0:
            self.timer = rospy.Timer(rospy.Duration(seconds), self.timer_callback)
            if print_new_time:
                self.cuprint("Next task time: " + str(seconds) + "s")    
        else:
            self.cuprint("No timeout monitoring on next task", warn=True)

    def timer_callback(self, msg):
        self.timer.shutdown()
        self.cuprint("Timed Out", warn=True)
        self.timed_out = True

    def timed_out(self):
        """ In objectives reference like: userdata.timeout_obj.timed_out """
        return self.timed_out