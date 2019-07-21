#!/usr/bin/env python
from __future__ import division
"""
Path Marker Task, attempts to center on the path, orient itself and update next task's prior
Objectives:
- Search
- Follow
---> Center on the path marker
---> Orient with the path marker
---> Update next task's prior
"""
from tasks.task import Task, Objective
from tasks.search import Search
import numpy as np
import rospy
import smach
import smach_ros
from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback
import actionlib
from actuator.srv import ActivateActuator

class Path(Task):
    name = "path"

    def __init__(self, path_num_str):
        self.name = "path" + path_num_str
        super(Path, self).__init__()
        self.init_objectives(path_num_str)
        self.link_objectives()

    def init_objectives(self, path_num_str):
        self.search = Search.from_bounding_box(self.get_prior_param(), "path", [0,0,0,0,0,1])
        self.follow = Follow(path_num_str)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Follow', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Follow', self.follow, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Follow(Objective):
    """
    Center, orient, update next task's prior
    """
    outcomes=['success', 'timed_out']

    def __init__(self, path_num_str):
        self.path_num_str = path_num_str
        self.client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        rospy.loginfo("...waiting for visual_servo server")
        self.client.wait_for_server()
        rospy.loginfo("....found visual servo server")
        self.centering_time = rospy.get_param("tasks/path"+path_num_str+"/centering_time")
        self.do_orientation = rospy.get_param("tasks/path"+path_num_str+"/do_orientation")
        self.feedback = False
        self.was_centered = False
        super(Follow, self).__init__(self.outcomes, "Follow")

    def feedback_callback(self, feedback):
        self.feedback = feedback.centered

    def execute(self, userdata):
        goal = VisualServoGoal()
        goal.target_class = "path"
        goal.camera = goal.DOWNCAM
        goal.target_frame = rospy.get_param("~robotname") +"/description/downcam_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_pixel_threshold = 10        # If inside a 20x20 square around the target pixel we'll be considered centered
        rospy.loginfo("...centering over path marker")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        
        while not rospy.is_shutdown() :
            if userdata.timeout_obj.timed_out:
                self.client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            if self.feedback:
                if self.was_centered:
                    break
                else:
                    rospy.sleep(self.centering_time)
                    self.was_centered = True
            else:
                self.was_centered = False
            rospy.sleep(1)
        rospy.loginfo("...centered over path marker")

        if self.do_orientation:
            # call to orientation server
            pass
        else:
            rospy.logwarn("...skipping orientation")
        
        userdata.outcome = "success"
        return "success"