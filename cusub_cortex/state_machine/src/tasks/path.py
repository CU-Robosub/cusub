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

class Path(Task):
    name = "path"

    def __init__(self, path_num_str):
        self.name = "path" + path_num_str
        super(Path, self).__init__()
        self.init_objectives(path_num_str)
        self.link_objectives()

    def init_objectives(self, path_num_str):
        path_topic = "cusub_perception/nodelet_perception/path" + path_num_str + "_seen"
        self.search = Search(self.get_prior_topic(), path_topic, darknet_cameras=[0,0,0,0,0,1]) # just downcam
        self.follow = Follow(path_num_str)

    def link_objectives(self):
        with self:
            # smach.StateMachine.add('Search', self.search, transitions={'found':'Follow', 'not_found':'manager'}, \
                # remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
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
        super(Follow, self).__init__(self.outcomes, "Follow")

    def execute(self, userdata):
        
        goal = VisualServoGoal()
        goal.target_class = "path"
        goal.target_frame = rospy.get_param("~robotname") +"/cusub_common/downcam"
        goal.visual_servo_type = goal.PROPORTIONAL
        rospy.loginfo("Sending Goal to Visual Servo Server")
        self.client.send_goal(goal)
        print(self.client.wait_for_result(rospy.Duration(180)))

        userdata.outcome = "success"
        return "success"