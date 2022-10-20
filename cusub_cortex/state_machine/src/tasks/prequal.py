#!/usr/bin/env python
from __future__ import division
"""
Startup Task, allows the man on the competition dock time to remove the tether from the vehicle before it starts its autonomous run.
Waits briefly at the surface before diving.
"""
import rospy
import smach
import smach_ros
from tasks.search import Search
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from detection_listener.listener import DetectionListener
from std_msgs.msg import Float64
from tasks.task import Task, Objective

class PrequalSm(Task):
    name = "Prequal"

    def __init__(self):
        super(PrequalSm, self).__init__(self.name) # become a state machine first

        self.listener = DetectionListener()

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        search_classes = ["vampire_cute"]
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())
        self.prequal = Prequal(self.name, self.listener)

    def link_objectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('Search', self.search, transitions={'found':'Pre', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Pre', self.prequal, transitions={'done':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Prequal(Objective):

    outcomes=['done']
    target_class_id = "vampire_cute"

    def __init__(self, task_name, listener):
        self.listener = listener
        name = task_name + "/PreQual"
        super(Prequal, self).__init__(self.outcomes, name)

    def execute(self, userdata):
        self.cuprint("Found the vampire")
        
        point1 = Point(-14.18,-1.45,-2)
        quaternion1 = Quaternion(2,2,2,2)
        pose1 = Pose()
        pose1.position = point1
        pose1.orientation = quaternion1

        self.toggle_waypoint_control(False)
        self.go_to_pose(pose1,userdata.timeout_obj)
        self.cuprint("reach 1")

        point2 = Point(-14.15,3.68,-2)
        quaternion2 = Quaternion(2,2,2,2)
        pose2 = Pose()
        pose2.position = point2
        pose2.orientation = quaternion2

        self.go_to_pose(pose2,userdata.timeout_obj)
        self.cuprint("reach 2")

        point3 = Point(-4.75,0,-2)
        quaternion3 = Quaternion(2,2,2,2)
        pose3 = Pose()
        pose3.position = point3
        pose3.orientation = quaternion3

        self.go_to_pose(pose3,userdata.timeout_obj)
        self.cuprint("reach 3")

        point4 = Point(-4.75,0,0)
        quaternion4 = Quaternion(2,2,2,2)
        pose4 = Pose()
        pose4.position = point4
        pose4.orientation = quaternion4

        self.go_to_pose(pose4,userdata.timeout_obj)
        self.cuprint("reach 4")


        userdata.outcome = "success"
        return "done"