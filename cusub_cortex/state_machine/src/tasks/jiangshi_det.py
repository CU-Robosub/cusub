#!/usr/bin/env python
from __future__ import division
"""
Jiangshi Buoy Task, attempts to bump into the jiangshi buoy
Objectives:
- Search
- Approach
- Slay
- Back Up
"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
# from geometry_msgs.msg import PoseStamped, Pose
import smach
import smach_ros
from detection_listener.listener import DetectionListener

# from std_msgs.msg import Float64
# from waypoint_navigator.srv import *

class Jiangshi(Task):
    name = "Jiangshi"

    def __init__(self):
        super(Jiangshi, self).__init__(self.name)

        # All Objectives share the same listener to gaurantee same data between objectives
        self.listener = DetectionListener() 
        
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        search_classes = ["vampire_cute"]
        self.search = Search(self.name, search_classes, self.listener, self.get_prior_param())
        # self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'manager', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            # smach.StateMachine.add('Slay', self.slay, transitions={'success':'manager', 'timed_out':'manager'}, \
                # remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})