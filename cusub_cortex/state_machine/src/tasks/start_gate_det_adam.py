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
        self.side_orbit = Side_Orbit(self.name, self.listener, clients)
        self.enter_side = Enter_Side(self.name, self.listener, clients)
        self.style = Style(self.name, self.listener, clients)

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

            smach.StateMachine.add('Center_Orbit', self.center_orbit, transitions={'centered':'Side_Orbit', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Side_Orbit', self.side_orbit, transitions={'ready_to_enter':'Enter_Side', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Enter_Side', self.enter_side, transitions={'entered':'Style', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Style', self.style, transitions={'finished':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
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
        self.right_increment_amt = rospy.get_param("tasks/start_gate/right_increment_amt")
        self.left_increment_amt = rospy.get_param("tasks/start_gate/left_increment_amt")
        self.spin_carrot = rospy.get_param("tasks/start_gate/spin_carrot_rads")

        self.before_enter_drive = 0

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

                self.az = bearing_arr[0]    # az for center pole when 3 poles detected
                self.el = bearing_arr[1]    # el for center pole when 3 poles detected
                self.mag = bearing_arr[2]   # mag for center pole when 3 poles detected
                self.az_l = bearing_arr[3]
                self.az_r = bearing_arr[4]
                self.mag_l = bearing_arr[5]
                self.mag_r = bearing_arr[6]

                # Transform pole az into the subframe
                self.trans_left_az = self.az_l - self.yaw_client.get_standard_state()
                self.trans_center_az = self.az - self.yaw_client.get_standard_state()
                self.trans_right_az = self.az_r - self.yaw_client.get_standard_state()
                # az between the left/center & right/center poles to enter
                self.enter_left_az = (((self.trans_left_az) + (self.trans_center_az)) / 2) + self.yaw_client.get_standard_state()
                self.enter_right_az = (((self.trans_right_az) + (self.trans_center_az)) / 2) + self.yaw_client.get_standard_state()
                
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
        self.yaw_client.set_setpoint(yaw_sp, loop=False)
        self.drive_client.set_setpoint(drive_sp, loop=False)
        self.strafe_client.set_setpoint(strafe_sp, loop=False)
        self.depth_client.set_setpoint(depth_sp, loop=False)

    def rospy_sleep(self, secs):
        rospy.sleep(secs)

    def action_func(self):
        self.yaw_setpoint = self.az
        if (0.95 * self.mag_target) <= self.mag <= (1.05 * self.mag_target):
            self.count += 1
            self.drive_setpoint += 0.01
            if self.count > self.count_target:
                self.cuprint("in position")
                self.drive_client.set_setpoint(self.drive_setpoint, loop=False)
                rospy.sleep(5)
                return True
        elif (self.mag_target - self.mag) > 0:
            if self.count:
                self.drive_setpoint += 0.15
            else:
                self.drive_setpoint += 0.20
        else:
            self.count = 0
            self.drive_setpoint -= 0.10
    
    def ret_bearing(self):
        az_l = 0
        az_r = 0

        mag_l = 0
        mag_r = 0

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
            [az_l, el_l, height_l, width_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
            [az_r, el_r, height_r, width_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
            mag = np.sqrt(height * width)
            mag_l = np.sqrt(height_l * width_l)
            mag_r = np.sqrt(height_r * width_r)
            
        return [az, el, mag, az_l, az_r, mag_l, mag_r]
    
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
        if (abs(abs(self.mag_l) - abs(self.mag_r)) < 8) and (abs(self.mag_l) > 95) and (abs(self.mag_r) > 95):
            self.count += 1
            if self.count > self.count_target:
                self.cuprint("centered")
                self.yaw_client.set_setpoint(self.az, loop=False)
                self.rospy_sleep(5)
                return True
        else:
            if self.mag_r > self.mag_l:
                self.strafe_setpoint = self.strafe_client.get_standard_state() - 0.45
            else:
                self.strafe_setpoint = self.strafe_client.get_standard_state() + 0.45


class Side_Orbit(Approach):
    outcomes = ['ready_to_enter', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Side_Orbit, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Side_Orbit, self).execute(userdata)
        if ret == "found_pole":
            return "ready_to_enter"
        else:
            return ret

    def action_func(self):
        if self.enter_right:
            self.yaw_setpoint = self.enter_right_az
            if (abs(self.trans_center_az) > 0.28):
                self.count += 1
            else:
                self.strafe_setpoint = self.strafe_client.get_standard_state() + 0.30
        else:
            self.yaw_setpoint = self.enter_left_az
            if (abs(self.trans_left_az) < 0.14) and (abs(self.trans_center_az) > 0.19):
                self.count += 1
            else:
                self.strafe_setpoint = self.strafe_client.get_standard_state() - 0.30

        if self.count > self.count_target:
            self.cuprint("ready_to_enter")
            self.rospy_sleep(5)
            return True


class Enter_Side(Approach):
    outcomes = ['entered', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Enter_Side, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Enter_Side, self).execute(userdata)
        if ret == "found_pole":
            return "entered"
        else:
            return ret

    def action_func(self):
        if self.count == 0:
            self.before_enter_drive = self.drive_client.get_standard_state()
            self.count += 1

        if self.drive_client.get_standard_state() > (self.before_enter_drive + 5):
            self.count += 1
            if self.count > self.count_target:
                self.cuprint("entered")
                self.rospy_sleep(5)
                return True
        else:
            if self.enter_right:
                self.yaw_setpoint = self.enter_right_az
            else:
                self.yaw_setpoint = self.enter_left_az
            self.drive_setpoint += 1




class Style(Approach):
    outcomes = ['finished', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Style, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        ret = super(Style, self).execute(userdata)
        if ret == "found_pole":
            return "finished"
        else:
            return ret

    def action_func(self):
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



class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")
