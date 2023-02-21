#!/usr/bin/env python
from __future__ import division
from turtle import delay

import rospy
import smach
import time
import smach_ros
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64
from tasks.task import Task, Objective
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tasks.pid_client import PIDClient

class squareSm(Task):
    name = "square"

    def __init__(self):
        super(squareSm, self).__init__(self.name)
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.square = square(self.name) ## ????????

    def link_objectives(self):
        with self: #we are state machine?
            smach.StateMachine.add('Pre', self.square, transitions={'done':'manager'}, \
                remapping={'timout_obj':'timout_obj', 'outcome':'outcome'})

class square(Objective):
    outcomes=['done']

    def __init__(self, task_name):
        name = task_name +"/square"
        self.yaw_client = PIDClient(name, "yaw")
        super(square, self).__init__(self.outcomes, name)

    def execute(self, userdata):
        print("new text")
        self.yaw_client.enable()
        
        point1 = Point(0,0,0)
        quaternion1 = Quaternion(2,2,2,2)
        self.yaw_client.set_setpoint(180, loop = False)
        # while self.yaw_client.get_standard_state() != self.yaw_client.set_setpoint.f.data:
        #     self.yaw_client.set_setpoint(180, loop = False)
        
        
        
        # q_new =   quaternion_from_euler(0,0,3.14)
        pose1 = Pose()
        # q_orig = quaternion_from_euler(0,0,0)
        #quaternion1 = quaternion_multiply(q_new, q_orig)
        pose1.position = point1
        pose1.orientation=quaternion1
        self.toggle_waypoint_control(False)
        self.go_to_pose(pose1,userdata.timeout_obj)
        self.cuprint("reach center")
        time.sleep(1)

        # point2 = Point(1, 0, 0)
        # quaternion2 = Quaternion(0,0,0,1)
        # pose2 = Pose()
        # pose2.position = point2
        # pose2.orientation = quaternion2
        # self.go_to_pose(pose2, userdata.timeout_obj)
        # self.cuprint("reach 2")
        # time.sleep(1)

        # point3 = Point(0,1,0)
        # quaternion3 = Quaternion(0,0,0,1)
        # pose3 = Pose()
        # pose3.position = point3
        # pose3.orientation=quaternion3
        # self.toggle_waypoint_control(False)
        # self.go_to_pose(pose3,userdata.timeout_obj)
        # self.cuprint("reach 3")
        # time.sleep(1)

        # point4 = Point(0,0,0)
        # quaternion4 = Quaternion(2,2,2,2)
        # pose4 = Pose()
        # pose4.position = point1
        # pose4.orientation=quaternion4
        # self.toggle_waypoint_control(False)
        # self.go_to_pose(pose4,userdata.timeout_obj)
        # self.cuprint("reach 4")
        # time.sleep(1)
        userdata.outcome = "success"
        return "done"