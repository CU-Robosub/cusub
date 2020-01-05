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
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener

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
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())
        self.approach = Approach(self.name, self.listener)
        self.revisit = Revisit(self.name, self.listener)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Revisit', self.revisit, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})                
            smach.StateMachine.add('Approach', self.approach, transitions={'success':'manager', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['success','timed_out', 'lost_object']

    def __init__(self, task_name, listener):
        super(Approach, self).__init__(self.outcomes, task_name + "/Approach")
        self.listener = listener
        self.yaw_client = PIDClient("yaw")
    
    def execute(self, userdata):
        self.smprint("executing")

        # We know we have a good detection
        # Find the dobject num

        self.yaw_client.enable()
        self.yaw_client.set_setpoint(0.0)
        
        """ Basically the visual servoing object needs to be able to 
        - switch certain PID loops to control other axes
        - decide which PID loops publish where (Yeah the visual servoing object could do this)
        """

        # Maybe I need a vs object to stop the waypoint navigator
        # Face toward the object --> Some assurance we're getting good hits?
        # Start the PID loops (should move us in the direction of the detections)
        # Publish the first setpoints

        while not rospy.is_shutdown():
            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"
                
        userdata.outcome = "success"
        return "success"

class Slay(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Slay, self).__init__(self.outcomes, task_name + "/Slay")


class Backup(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Backup, self).__init__(self.outcomes, task_name + "/Backup")

class Revisit(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Revisit, self).__init__(self.outcomes, task_name + "/Revisit")