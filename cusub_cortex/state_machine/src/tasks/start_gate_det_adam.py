#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
"""
StartGate Task, attempts to go through right side of StartGate
Objectives:
- Search
- Approach 1 Pole
- Approach 2 Poles
- Approach 3 Poles
- Center Orbit
- Center Strafe
- Enter / Style
"""

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
        yaw_client = PIDClient(self.name, "yaw")
        clients = {
            "drive_client": drive_client,
            "strafe_client": strafe_client,
            "depth_client": depth_client,
            "yaw_client": yaw_client
        }

        search_classes = ["start_gate_pole"]

        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())

        self.approach = Approach(self.name, self.listener, clients)
        self.approach_2_poles = Approach_2_Poles(self.name, self.listener, clients)
        self.approach_3_poles = Approach_3_Poles(self.name, self.listener, clients)

        self.center_orbit = Center_Orbit(self.name, self.listener, clients)
        self.center_strafe = Center_Strafe(self.name, self.listener, clients)
        self.enter_with_style = Enter_With_Style(self.name, self.listener, clients)

        self.retrace = Retrace(self.name, self.listener)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach', self.approach, transitions={'found_pole':'Approach_2_Poles', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_2_Poles', self.approach_2_poles, transitions={'found_3rd_pole':'Approach_3_Poles', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_3_Poles', self.approach_3_poles, transitions={'in_position':'Center_Orbit', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Center_Orbit', self.center_orbit, transitions={'centered':'Center_Strafe', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Center_Strafe', self.center_strafe, transitions={'ready_to_enter':'Enter_With_Style', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Enter_With_Style', self.enter_with_style, transitions={'finished':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})


class Approach(Objective):
    outcomes = ['found_pole', 'timed_out', 'lost_object']
    num_of_poles = 1

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.target_class_id = "start_gate_pole"
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.depth_client = clients["depth_client"]
        self.yaw_client = clients["yaw_client"]
        self.listener = listener

        self.retrace_timeout = rospy.get_param("tasks/start_gate/retrace_timeout", 30)

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/mag_target")
        self.enter_right = rospy.get_param("tasks/start_gate/enter_right")
        self.enter_right_strafe_amt = rospy.get_param("tasks/start_gate/enter_right_strafe_amt")
        self.enter_left_strafe_amt = rospy.get_param("tasks/start_gate/enter_left_strafe_amt")
        self.spin_carrot = rospy.get_param("tasks/start_gate/spin_carrot_rads")

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        # Find start_gate pole dobject number and check for errors
        self.dobj_nums = self.listener.query_class(self.target_class_id)
        
        if len(self.dobj_nums) > 0: # Check if more than 1 instance of target class
            self.cuprint(str(len(self.dobj_nums)) + " " + self.target_class_id + " classe(s) detected!", warn=True)
        elif not self.dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + str(self.target_class_id) + " classes found in listener?", warn=True)
            return "lost_object"

        # DOBJECT NUMBER
        dobj_num = self.dobj_nums[0]
        if len(self.dobj_nums) == 3:
            self.eval_startgate_poles()
            dobj_num = self.pole_c

        watchdog_timer = Timeout(self.name + u"/Rétrace Watchdog".encode("utf-8"))

        # Enable the PID loops
        self.PID_enable()

        watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        print("") # overwritten by servoing status
        while not rospy.is_shutdown():
            seeing_num_of_poles = self.listener.query_class(self.target_class_id)

            if self.listener.check_new_dv(dobj_num) or (self.num_of_poles == 3):
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

                bearing_arr = self.ret_bearing()

                self.az = bearing_arr[0]
                self.el = bearing_arr[1]
                self.mag = bearing_arr[2]
                
                self.drive_setpoint = self.drive_client.get_standard_state()
                self.strafe_setpoint = self.strafe_client.get_standard_state()
                self.yaw_setpoint = self.yaw_client.get_standard_state()

                self.cuprint("CHECK POSITION: " + str(self.count) + " / " + str(self.count_target), warn=True, print_prev_line=True)

                if self.action_func():
                    break

            elif watchdog_timer.timed_out:
                self.PID_disable()
                self.cuprint("Retrace watchdog timed out.", warn=True)
                return "lost object"

            if userdata.timeout_obj.timed_out:
                watchdog_timer.timer.shutdown()
                userdata.outcome = "timed_out"
                return "not_found"

            # Detected 2nd Pole, break
            if self.num_of_poles == 1:
                if len(seeing_num_of_poles) > 1:
                    self.cuprint("Detected 2nd Pole")
                    break

            # Detected 3rd Pole
            if self.num_of_poles == 2:
                if len(seeing_num_of_poles) == 3:
                    self.cuprint("Detected 3rd Pole")
                    break
                
            self.PID_all_set_setpoint(self.drive_setpoint, self.strafe_setpoint, -1.5, self.yaw_setpoint)
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.PID_disable()
        return "found_pole"

    def PID_enable(self):
        self.drive_client.enable()
        self.strafe_client.enable()
        self.depth_client.enable()
        self.yaw_client.enable()

    def PID_disable(self):
        self.drive_client.disable()
        self.strafe_client.disable()
        self.depth_client.disable()
        self.yaw_client.disable()

    def PID_all_set_setpoint(self, drive_sp, strafe_sp, depth_sp, yaw_sp):
        self.drive_client.set_setpoint(drive_sp, loop=False)
        self.strafe_client.set_setpoint(strafe_sp, loop=False)
        self.depth_client.set_setpoint(depth_sp, loop=False)
        self.yaw_client.set_setpoint(yaw_sp, loop=False)

    def rospy_sleep(self, secs):
        rospy.sleep(secs)

    def action_func(self):
        self.yaw_setpoint = self.az
        # If within 10% of magnitude target
        if (0.9 * self.mag_target) <= self.mag <= (1.1 * self.mag_target):
            self.count += 1
            self.drive_setpoint += 0.15
            if self.count > self.count_target:
                self.cuprint("in position")
                self.drive_client.set_setpoint(self.drive_setpoint, loop=False)
                rospy.sleep(5)
                return True
        elif (self.mag_target - self.mag) > 0:
            if self.count:
                self.drive_setpoint += 0.2
            else:
                self.drive_setpoint += 0.4
        else:
            self.count = 0
            self.drive_setpoint -= 0.3
    
    def ret_bearing(self):
        if self.num_of_poles == 1:
            [az, el, height, width] = self.listener.get_avg_bearing(self.dobj_nums[0], num_dv=5)
            mag = np.sqrt(height * width)

        if self.num_of_poles == 2:
            [az_0, el_0, height_0, width_0] = self.listener.get_avg_bearing(self.dobj_nums[0], num_dv=5)
            [az_1, el_1, height_1, width_1] = self.listener.get_avg_bearing(self.dobj_nums[1], num_dv=5)
            
            az = ((az_0 + az_1)/2)
            el = ((el_0 + el_1)/2)
            mag = np.sqrt(((height_0 * width_0) + (height_1 * width_1)) / 2)

        if self.num_of_poles == 3:
            [az, el, height, width] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)
            mag = np.sqrt(height * width)
            
        return [az, el, mag]
    
    def eval_startgate_poles(self):
        [az_0, el_0, height_0, width_0] = self.listener.get_avg_bearing(self.dobj_nums[0], num_dv=5)
        [az_1, el_1, height_1, width_1] = self.listener.get_avg_bearing(self.dobj_nums[1], num_dv=5)
        [az_2, el_2, height_2, width_2] = self.listener.get_avg_bearing(self.dobj_nums[2], num_dv=5)

        az_dict = {
            float(az_0): self.dobj_nums[0],
            float(az_1): self.dobj_nums[1],
            float(az_2): self.dobj_nums[2]
        }

        az_output = sorted(az_dict, key = lambda x:float(x))

        self.pole_l = self.dobj_nums[az_dict.get(az_output[2])]
        self.pole_r = self.dobj_nums[az_dict.get(az_output[0])]
        self.pole_c = self.dobj_nums[az_dict.get(az_output[1])]


class Approach_2_Poles(Approach):
    outcomes = ['found_3rd_pole', 'timed_out', 'lost_object']
    num_of_poles = 2

    def __init__(self, task_name, listener, clients):
        super(Approach_2_Poles, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Approach_2_Poles, self).execute(userdata)
        if ret == "found_pole":
            return "found_3rd_pole"
        else:
            return ret


class Approach_3_Poles(Approach):
    outcomes = ['in_position', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Approach_3_Poles, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Approach_3_Poles, self).execute(userdata)
        if ret == "found_pole":
            return "in_position"
        else:
            return ret


class Center_Orbit(Approach):
    outcomes = ['centered', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Center_Orbit, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Center_Orbit, self).execute(userdata)
        if ret == "found_pole":
            return "centered"
        else:
            return ret

    def action_func(self):
        self.yaw_setpoint = self.az
        if -0.03 < self.az < 0.03:
            self.count += 1
            if self.count > self.count_target:
                self.cuprint("centered")
                self.yaw_client.set_setpoint(0, loop=False)
                return True
        elif abs(self.az) < 0.15:
            if self.az < 0:
                self.strafe_setpoint = self.strafe_client.get_standard_state() + 0.25
            else:
                self.strafe_setpoint = self.strafe_client.get_standard_state() - 0.25
        else:
            self.count = 0
            if self.az < 0:
                self.strafe_setpoint = self.strafe_client.get_standard_state() + 0.60
            else:
                self.strafe_setpoint = self.strafe_client.get_standard_state() - 0.60


class Center_Strafe(Approach):
    outcomes = ['ready_to_enter', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Center_Strafe, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Center_Strafe, self).execute(userdata)
        if ret == "found_pole":
            return "ready_to_enter"
        else:
            return ret

    def action_func(self):
        self.yaw_setpoint = 0
        if self.enter_right:
            self.strafe_setpoint = self.strafe_client.get_standard_state() + self.enter_right_strafe_amt
        else:
            self.strafe_setpoint = self.strafe_client.get_standard_state() + self.enter_left_strafe_amt
        
        self.count += 1
        if self.count > self.count_target:
            self.cuprint("ready_to_enter")
            self.yaw_client.set_setpoint(0, loop=False)
            self.rospy_sleep(10)
            return True


class Enter_With_Style(Approach):
    outcomes = ['finished', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Enter_With_Style, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Enter_With_Style, self).execute(userdata)
        if ret == "found_pole":
            return "finished"
        else:
            return ret

    def action_func(self):
        if self.mag_target <= self.mag and 1.0 <= abs(self.az):
            self.count += 1
            if self.count > self.count_target:
                self.cuprint("entered startgate")

                radians_turned = 0
                current_yaw = self.yaw_client.get_standard_state()
                while radians_turned < 4*np.pi and not rospy.is_shutdown():
                    previous_yaw = current_yaw
                    current_yaw = self.yaw_client.get_standard_state() + self.spin_carrot
                    diff_yaw = current_yaw - previous_yaw
                    if abs(diff_yaw) < np.pi:
                        radians_turned += diff_yaw
                    self.yaw_client.set_setpoint(current_yaw, loop=False)

                self.cuprint("finished")

                return True
        else:
            if self.count:
                self.drive_setpoint += 0.4
            else:
                self.drive_setpoint += 0.6


class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")
