#!/usr/bin/env python
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

from tasks.task import Task, Objective
from tasks.search import Search
from tasks.pid_client import PIDClient
import tf
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import copy
from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoActionFeedback
import actionlib
from std_msgs.msg import Float64
from waypoint_navigator.srv import *


class StartGate(Task):
    name = "start_gate"

    def __init__(self):
        super(StartGate, self).__init__(self.name)

        # All Objectives share the same listener to guarantee same data between objectives
        self.listener = DetectionListener()

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        search_classes = ["start_gate_pole"]
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())
        self.approach_1_pole = Approach_1_Pole(self.name, self.listener)
        self.approach_2_poles = Approach_2_Poles(self.name, self.listener)
        self.approach_3_poles = Approach_3_Poles(self.name, self.listener)

        self.center_strafe = Center_Strafe(self.name, self.listener)
        self.penetrate_with_style = Penetrate_With_Style(self.name, self.listener)

        self.slay = Slay(self.name)
        self.revisit = Revisit(self.name, self.listener)



    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach_1_Pole', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_1_Pole', self.approach_1_pole, transitions={'found_2nd_pole':'Approach_2_Poles', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_2_Poles', self.approach_2_poles, transitions={'found_3rd_pole':'Approach_3_Poles', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_3_Poles', self.approach_3_poles, transitions={'in_position':'Center_Strafe', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Center_Strafe', self.center_strafe, transitions={'centered':'Penetrate_With_Style', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Penetrate_With_Style', self.penetrate_with_style, transitions={'finished':'manager', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})




            smach.StateMachine.add('Revisit', self.revisit, transitions={'found':'Approach_1_Pole', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Slay', self.slay, transitions={'slayed':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            # FILL IN REST OF OBJECTIVES


class Approach_1_Pole(Objective):
    outcomes = ['found_2nd_pole', 'timed_out', 'lost_object']

    target_class_id = "start_gate_pole"

    def __init__(self, task_name, listener):
        name = task_name + "/Approach_1_Pole"
        super(Approach_1_Pole, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive") #, "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth")

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/mag_target")
        self.elev_target = rospy.get_param("tasks/start_gate/elev_target")
        self.elev_thresh = rospy.get_param("tasks/start_gate/elev_thresh")

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 0: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classe(s) detected!", warn=True)
            self.cuprint("selecting the first", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[0]
        self.cuprint("located " + self.target_class_id + "'s dobject num: " + str(dobj_num))

        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(-1.5)
        self.in_position_state = None

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az, el, mag] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                mag = np.sqrt(mag)

                self.yaw_client.set_setpoint(az, loop=False)
                if abs(self.mag_target - mag) < 0.5*self.mag_target:
                    self.cuprint("[MAGNITUDE: " + str(mag) + "] - " + "[AZ: " + str(az) + "] - " +  "[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    self.count += 1
                    if self.count == (self.count_target / 3):
                        self.in_position_state = self.drive_client.get_standard_state()
                        self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    # Stop Sub
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        self.yaw_client.set_setpoint(0, loop=False)
                        self.drive_client.increment_setpoint(0.15, loop=False)
                        rospy.sleep(5)
                        break
                elif self.mag_target - mag > 0:
                    self.cuprint("mag: " + str(mag))
                    # Move sub forward
                    if self.count:
                        self.drive_client.increment_setpoint(0.2, loop=False)
                    else:
                        self.drive_client.increment_setpoint(0.6, loop=False)
                else:
                    # Not within threshold
                    self.count = 0
                    self.drive_client.increment_setpoint(-0.2, loop=False)

                self.listener.clear_new_dv_flag(dobj_num)

            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"

            # Detected 2nd Pole
            if len(dobj_nums) > 1:
                self.cuprint("Detected 2nd Pole")
                break
                
            self.depth_client.set_setpoint(-1.5, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "found_2nd_pole"


class Approach_2_Poles(Objective):
    outcomes = ['found_3rd_pole', 'timed_out', 'lost_object']

    target_class_id = "start_gate_pole"

    def __init__(self, task_name, listener):
        name = task_name + "/Approach_2_Poles"
        super(Approach_2_Poles, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive") #, "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth")

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/mag_target")
        self.elev_target = rospy.get_param("tasks/start_gate/elev_target")
        self.elev_thresh = rospy.get_param("tasks/start_gate/elev_thresh")


    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.cuprint("selecting the second", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[1]
        self.cuprint("located " + self.target_class_id + "'s dobject num: " + str(dobj_num))

        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(-1.5)
        self.in_position_state = None

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az_1, el_1, mag_1] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
                [az_2, el_2, mag_2] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                az = ((az_1 + az_2)/2)
                el = ((el_1 + el_2)/2)
                mag = np.sqrt(((mag_1 + mag_2)/2))

                self.yaw_client.set_setpoint(az, loop=False)
                if abs(self.mag_target - mag) < 0.5*self.mag_target:
                    self.cuprint("[MAGNITUDE: " + str(mag) + "] - " + "[AZ: " + str(az) + "] - " +  "[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    self.count += 1
                    if self.count == (self.count_target / 3):
                        self.in_position_state = self.drive_client.get_standard_state()
                        self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    # Stop Sub
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        self.yaw_client.set_setpoint(0, loop=False)
                        self.drive_client.increment_setpoint(0.15, loop=False)
                        rospy.sleep(5)
                        break
                elif self.mag_target - mag > 0:
                    self.cuprint("mag: " + str(mag))
                    # Move sub forward
                    if self.count:
                        self.drive_client.increment_setpoint(0.2, loop=False)
                    else:
                        self.drive_client.increment_setpoint(0.6, loop=False)
                else:
                    # Not within threshold
                    self.count = 0
                    self.drive_client.increment_setpoint(-0.2, loop=False)

                self.listener.clear_new_dv_flag(dobj_num)

            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"

            # Detected 3rd Pole
            if len(dobj_nums) == 3:
                self.cuprint("Detected 3rd Pole")
                break
                
            self.depth_client.set_setpoint(-0.8, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "found_3rd_pole"


class Approach_3_Poles(Objective):
    outcomes = ['in_position', 'timed_out', 'lost_object']

    target_class_id = "start_gate_pole"

    def __init__(self, task_name, listener):
        name = task_name + "/Approach_3_Poles"
        super(Approach_3_Poles, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive") #, "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth")

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/center_pole_target")
        self.elev_target = rospy.get_param("tasks/start_gate/elev_target")
        self.elev_thresh = rospy.get_param("tasks/start_gate/elev_thresh")


    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        self.eval_startgate_poles(dobj_nums)
        dobj_num = self.pole_c

        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.depth_client.set_setpoint(-1.5)
        self.in_position_state = None

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az_l, el_l, mag_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
                [az_r, el_r, mag_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
                [az_c, el_c, mag_c] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)
                
                az = az_c
                el = el_c
                mag = np.sqrt(mag_c)

                self.yaw_client.set_setpoint(az, loop=False)
                if abs(self.mag_target - mag) < 0.5*self.mag_target:
                    self.cuprint("[MAGNITUDE: " + str(mag) + "] - " + "[AZ: " + str(az) + "] - " +  "[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    self.count += 1
                    if self.count == (self.count_target / 3):
                        self.in_position_state = self.drive_client.get_standard_state()
                        self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    # Stop Sub
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        self.yaw_client.set_setpoint(0, loop=False)
                        self.in_position_state = self.drive_client.get_standard_state()
                        self.drive_client.increment_setpoint(0.15, loop=False)
                        rospy.sleep(10)
                        break
                elif self.mag_target - mag > 0:
                    self.cuprint("mag: " + str(mag))
                    # Move sub forward
                    if self.count:
                        self.drive_client.increment_setpoint(0.2, loop=False)
                    else:
                        self.drive_client.increment_setpoint(0.6, loop=False)
                else:
                    # Not within threshold
                    self.count = 0
                    self.drive_client.increment_setpoint(-0.2, loop=False)

                self.listener.clear_new_dv_flag(dobj_num)
                    
            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"
                
            self.depth_client.set_setpoint(-0.8, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "in_position"


    def eval_startgate_poles(self, dobj_nums):
        [az_0, el_0, mag_0] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
        [az_1, el_1, mag_1] = self.listener.get_avg_bearing(dobj_nums[1], num_dv=5)
        [az_2, el_2, mag_2] = self.listener.get_avg_bearing(dobj_nums[2], num_dv=5)

        if az_0 > az_1 > az_2:
            self.pole_l = dobj_nums[0]
            self.pole_r = dobj_nums[2]
            self.pole_c = dobj_nums[1]

        elif az_2 > az_1 > az_0:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[0]
            self.pole_c = dobj_nums[1]

        elif az_0 > az_2 > az_1:
            self.pole_l = dobj_nums[0]
            self.pole_r = dobj_nums[1]
            self.pole_c = dobj_nums[2]

        elif az_1 > az_2 > az_0:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[0]
            self.pole_c = dobj_nums[2]

        elif az_1 > az_0 > az_2:
            self.pole_l = dobj_nums[1]
            self.pole_r = dobj_nums[2]
            self.pole_c = dobj_nums[0]

        else:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[1]
            self.pole_c = dobj_nums[0]


class Center_Strafe(Objective):
    outcomes = ['centered', 'timed_out', 'lost_object']

    target_class_id = "start_gate_pole"

    def __init__(self, task_name, listener):
        name = task_name + "/Center_Strafe"
        super(Center_Strafe, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive") #, "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth")

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/center_pole_target")
        self.elev_target = rospy.get_param("tasks/start_gate/elev_target")
        self.elev_thresh = rospy.get_param("tasks/start_gate/elev_thresh")

        # Strafe info
        self.orbit_right = rospy.get_param("tasks/start_gate/strafe_right")
        self.strafe_carrot = rospy.get_param("tasks/start_gate/strafe_carrot")
        self.strafe_state = None
        rospy.Subscriber("cusub_common/motor_controllers/pid/strafe/setpoint", Float64, self.strafe_callback)
        self.strafe_pub = rospy.Publisher("cusub_common/motor_controllers/pid/strafe/setpoint", Float64, queue_size=10)

    def strafe_callback(self, msg):
        self.strafe_state = msg.data

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 2: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        self.eval_startgate_poles(dobj_nums)
        dobj_num = self.pole_c

        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.depth_client.set_setpoint(-1.5)
        self.in_position_state = None

        self.cuprint("strafe-ing [right: " + str(self.orbit_right) + "]")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az_l, el_l, mag_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
                [az_r, el_r, mag_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
                [az_c, el_c, mag_c] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)
                
                # az = az_c
                # el = el_c
                # mag = np.sqrt(mag_c)

                az_diff = 0
                if self.orbit_right:
                    az_diff = abs(abs(az_r) - abs(az_c))
                else:
                    az_diff = abs(abs(az_l) - abs(az_c))

                # Adjust strafe with carrot on current strafe
                strafe_msg = Float64()
                strafe_set = 0.0 if self.strafe_state is None else self.strafe_state
                if az_diff <= 0.3:
                    self.cuprint("[AZ_DIFF: " + str(az_diff) + "] - " +  "[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    if self.count == 1:
                        self.in_position_state = self.drive_client.get_standard_state()
                    if self.in_position_state is not None:
                        self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    self.count += 1
                    # Stop Sub
                    if self.count > self.count_target:
                        self.cuprint("centered")
                        self.yaw_client.set_setpoint(0, loop=False)
                        self.drive_client.increment_setpoint(0.15, loop=False)
                        rospy.sleep(5)
                        break
                elif az_diff > 0.3:
                    self.cuprint("az_diff: " + str(az_diff))
                    # Strafe
                    if self.orbit_right:
                        strafe_msg.data = strafe_set + self.strafe_carrot
                    else:
                        strafe_msg.data = strafe_set - self.strafe_carrot
                    self.strafe_pub.publish(strafe_msg)
                else:
                    # Not within threshold
                    self.count = 0

                self.listener.clear_new_dv_flag(dobj_num)

                    
            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"
                
            self.depth_client.set_setpoint(-0.8, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "centered"


    def eval_startgate_poles(self, dobj_nums):
        [az_0, el_0, mag_0] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
        [az_1, el_1, mag_1] = self.listener.get_avg_bearing(dobj_nums[1], num_dv=5)
        [az_2, el_2, mag_2] = self.listener.get_avg_bearing(dobj_nums[2], num_dv=5)

        if az_0 > az_1 > az_2:
            self.pole_l = dobj_nums[0]
            self.pole_r = dobj_nums[2]
            self.pole_c = dobj_nums[1]

        elif az_2 > az_1 > az_0:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[0]
            self.pole_c = dobj_nums[1]

        elif az_0 > az_2 > az_1:
            self.pole_l = dobj_nums[0]
            self.pole_r = dobj_nums[1]
            self.pole_c = dobj_nums[2]

        elif az_1 > az_2 > az_0:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[0]
            self.pole_c = dobj_nums[2]

        elif az_1 > az_0 > az_2:
            self.pole_l = dobj_nums[1]
            self.pole_r = dobj_nums[2]
            self.pole_c = dobj_nums[0]

        else:
            self.pole_l = dobj_nums[2]
            self.pole_r = dobj_nums[1]
            self.pole_c = dobj_nums[0]


class Penetrate_With_Style(Objective):
    outcomes = ['finished', 'timed_out', 'lost_object']

    def __init__(self, task_name, listener):
        name = task_name + "/Penetrate_With_Style"
        super(Penetrate_With_Style, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive") #, "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth")

        self.spin_carrot = rospy.get_param('tasks/start_gate/spin_carrot_rads', 0.3)
        self.yaw_pub = rospy.Publisher("cusub_common/motor_controllers/pid/yaw/setpoint", Float64, queue_size=10)
        rospy.Subscriber("cusub_common/motor_controllers/pid/yaw/state", Float64, self.yaw_callback)

    def yaw_callback(self, msg):
        self.current_yaw = msg.data

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.depth_client.set_setpoint(-0.8, loop=False)

        self.cuprint("Penetrating Startgate")
        self.drive_client.increment_setpoint(2.5, loop=False)
        rospy.sleep(10)

        self.cuprint("Stylin'")
        radians_turned = 0
        yaw_set = Float64()
        yaw_set.data = self.current_yaw
        while radians_turned < 4*np.pi and not rospy.is_shutdown():
            prev_set = yaw_set.data
            yaw_set.data = self.current_yaw + self.spin_carrot
            yaw_set_diff = yaw_set.data - prev_set
            if abs(yaw_set_diff) < np.pi: # check for wrapping
                radians_turned += yaw_set_diff
            self.yaw_pub.publish(yaw_set)

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "finished"


class Slay(Objective):
    outcomes = ['slayed', 'timed_out']

    def __init__(self, task_name):
        name = task_name + "/Slay"
        super(Slay, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/start_gate/slay_carrot_dist")

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)
        self.drive_client.enable()
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.set_setpoint(cur_state + self.carrot_dist)
        rospy.sleep(10)
        self.cuprint("slayed")
        self.drive_client.disable()
        return "slayed"


# TODO
class Revisit(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Revisit, self).__init__(self.outcomes, task_name + "/Revisit")