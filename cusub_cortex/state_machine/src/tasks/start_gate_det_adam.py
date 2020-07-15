#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
"""
StartGate Task, attempts to go through right side of StartGate
Objectives:
- Search
- Approach 1 Pole
- Approach 2 Poles
- Center Orbit
- Penetrate
- Style
"""

# from tasks.task import Task, Objective
# from tasks.search import Search
# from tasks.pid_client import PIDClient
# import tf
# import numpy as np
# import rospy
# from geometry_msgs.msg import PoseStamped, Pose
# import smach
# import smach_ros
# from detection_listener.listener import DetectionListener
# import copy
# import actionlib
from std_msgs.msg import Float64
from waypoint_navigator.srv import *


from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose
from localizer_msgs.msg import Detection
from actuator.srv import ActivateActuator
from cusub_print.cuprint import bcolors


class StartGate(Task):
    name = "start_gate"

    def __init__(self):
        super(StartGate, self).__init__(self.name)

        # All Objectives share the same listener to guarantee same data between objectives
        self.listener = DetectionListener()

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive")
        strafe_client = PIDClient(self.name, "strafe")
        depth_client = PIDClient(self.name, "depth")
        clients = {
            "drive_client": drive_client,
            "strafe_client": strafe_client,
            "depth_client": depth_client
        }

        search_classes = ["start_gate_pole"]

        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())
        self.approach = Approach(self.name, self.listener, clients)
        self.retrace = Retrace(self.name, self.listener)



    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach', self.approach, transitions={'found_pole':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})



class Approach(Objective):
    outcomes = ['found_pole', 'timed_out', 'lost_object']
    
    target_class_id = "start_gate_pole"

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.depth_client = clients["depth_client"]
        self.listener = listener

        self.retrace_timeout = rospy.get_param("tasks/start_gate/retrace_timeout", 15)

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/center_pole_target")


    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 0: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classe(s) detected!", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + str(self.target_class_id) + " classes found in listener?", warn=True)
            return "lost_object"

        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[0]
        if len(dobj_nums) == 3:
            self.eval_startgate_poles(dobj_nums)
            dobj_num = self.pole_c

        watchdog_timer = Timeout(self.name + u"/Rétrace Watchdog".encode("utf-8"))

        # Enable the PID loops
        self.drive_client.enable()
        self.strafe_client.enable()
        self.depth_client.enable()

        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(-1.5)

        watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        print("") # overwritten by servoing status
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)
                
                drive_setpoint = self.drive_client.get_standard_state() + 0.8
                self.cuprint(str(self.count))

                self.count += 1

                if self.count > self.count_target:
                    break

            elif watchdog_timer.timed_out:
                self.drive_client.disable()
                self.strafe_client.disable()
                self.depth_client.disable()
                self.cuprint("Retrace watchdog timed out.", warn=True)
                return "lost object"

            if userdata.timeout_obj.timed_out:
                watchdog_timer.timer.shutdown()
                userdata.outcome = "timed_out"
                return "not_found"
                
            self.drive_client.set_setpoint(drive_setpoint, loop=False)
            self.depth_client.set_setpoint(-1.5, loop=False)
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.drive_client.disable()
        self.depth_client.disable()
        return "found_pole"


    def eval_startgate_poles(self, dobj_nums):
        [az_0, el_0, height_0, width_0] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
        [az_1, el_1, height_1, width_1] = self.listener.get_avg_bearing(dobj_nums[1], num_dv=5)
        [az_2, el_2, height_2, width_2] = self.listener.get_avg_bearing(dobj_nums[2], num_dv=5)

        mag_0 = height_0 * width_0
        mag_1 = height_1 * width_1
        mag_2 = height_2 * width_2

        az_dict = {
            float(az_0): dobj_nums[0],
            float(az_1): dobj_nums[1],
            float(az_2): dobj_nums[2]
        }

        az_output = sorted(az_dict, key = lambda x:float(x))

        self.pole_l = dobj_nums[az_dict.get(az_output[2])]
        self.pole_r = dobj_nums[az_dict.get(az_output[0])]
        self.pole_c = dobj_nums[az_dict.get(az_output[1])]


# TODO
class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")