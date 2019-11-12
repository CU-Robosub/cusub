#!/usr/bin/env python
from __future__ import division
"""
StartGate Task, attempts to go through the start gate.
Receives a pose of the start gate and adjusts the pose according to the smaller side of the gate in order to maximize points.
---
Objectives:
1) Search (based on prior)
2) Attack (goes behind gate based on arg distBehindGate)

"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Bool, Float64
import numpy as np
from waypoint_navigator.srv import ToggleControl
import tf
import copy

class Prequal_2020(Task):
    name = "prequal_2020"

    def __init__(self):
        super(Prequal_2020, self).__init__() # become a state machine first
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.attack = Attack()

    def link_objectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('Attack', self.attack, transitions={'success':'manager','timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Attack(Objective):
    """
    Tell the sub to go through the gate
    """

    outcomes=['success', 'timed_out']

    def __init__(self):
        rospy.loginfo("Loading attack")
        super(Attack, self).__init__(self.outcomes, "Attack")
        pose1 = Pose()
        pose1.position.x = 1
        pose1.position.y = 1
        pose1.position.z = 1

        pose2 = Pose()
        pose2.position.x = 1
        pose2.position.y = 1
        pose2.position.z = 1

        pose3 = Pose()
        pose3.position.x = 1
        pose3.position.y = 1
        pose3.position.z = 1

        pose4 = Pose()
        pose4.position.x = 1
        pose4.position.y = 1
        pose4.position.z = 1

        pose5 = Pose()
        pose5.position.x = 1
        pose5.position.y = 1
        pose5.position.z = 1

        self.target_poses = [pose1, pose2, pose3, pose4, pose5]

    def execute(self, userdata):
        rospy.loginfo("Executing Attack")
        for i in range(len(self.target_poses)):
            if self.go_to_pose(self.target_poses[i], userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "timed_out"
                else: # Replan has been requested loop again
                    pass
            else:
                self.update_next_priors("start_gate")
                userdata.outcome = "success"
                return "success"