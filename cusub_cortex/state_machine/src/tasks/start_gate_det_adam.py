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

            smach.StateMachine.add('Center_Orbit', self.center_orbit, transitions={'centered':'Enter_With_Style', 'timed_out':'manager', 'lost_object':'Retrace'}, \
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

        self.retrace_timeout = rospy.get_param("tasks/start_gate/retrace_timeout", 15)

        seconds = rospy.get_param("tasks/start_gate/seconds_in_postion", 2)
        self.rate = rospy.get_param("tasks/start_gate/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0

        self.mag_target = rospy.get_param("tasks/start_gate/mag_target")
        self.enter_right = rospy.get_param("tasks/start_gate/enter_right")
        self.spin_carrot = rospy.get_param("tasks/start_gate/spin_carrot_rads")


    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        self.cuprint("num_of_poles: " + str(self.num_of_poles), warn=True)

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        self.target_class_print(dobj_nums)

        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[0]
        if len(dobj_nums) == 3:
            self.eval_startgate_poles(dobj_nums)
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

                bearing_arr = self.ret_bearing(dobj_nums)

                az = bearing_arr[0]
                el = bearing_arr[1]
                mag = bearing_arr[2]
                
                drive_setpoint = self.drive_client.get_standard_state()
                strafe_setpoint = self.strafe_client.get_standard_state()

                self.cuprint("CHECK POSITION: " + str(self.count) + " / " + str(self.count_target), print_prev_line=True)

                if (0.9 * self.mag_target) <= mag <= (1.1 * self.mag_target):
                    self.count += 1
                    drive_setpoint += 0.01
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        self.drive_client.set_setpoint(drive_setpoint, loop=False)
                        rospy.sleep(5)
                        break
                elif (self.mag_target - mag) > 0:
                    if self.count:
                        drive_setpoint += 0.3
                    else:
                        drive_setpoint += 0.6
                else:
                    self.count = 0
                    drive_setpoint -= 0.2

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
                
            self.PID_all_set_setpoint(drive_setpoint, strafe_setpoint, -1.5, az)
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
        self.drive_client.enable()
        self.strafe_client.enable()
        self.depth_client.enable()
        self.yaw_client.enable()

    def PID_all_set_setpoint(self, drive_sp, strafe_sp, depth_sp, yaw_sp):
        self.drive_client.set_setpoint(drive_sp, loop=False)
        self.strafe_client.set_setpoint(strafe_sp, loop=False)
        self.depth_client.set_setpoint(depth_sp, loop=False)
        self.yaw_client.set_setpoint(yaw_sp, loop=False)

    def target_class_print(self, dobj_nums):
        if len(dobj_nums) > 0: # Check if more than 1 instance of target class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classe(s) detected!", warn=True)
        elif not dobj_nums: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + str(self.target_class_id) + " classes found in listener?", warn=True)
            return "lost_object"

    
    def ret_bearing(self, dobj_nums):
        az_l = az_r = az_c = -1

        if self.num_of_poles == 1:
            [az, el, height, width] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
            mag = np.sqrt(height * width)

        if self.num_of_poles == 2:
            [az_0, el_0, height_0, width_0] = self.listener.get_avg_bearing(dobj_nums[0], num_dv=5)
            [az_1, el_1, height_1, width_1] = self.listener.get_avg_bearing(dobj_nums[1], num_dv=5)
            
            az = ((az_0 + az_1)/2)
            el = ((el_0 + el_1)/2)
            
            mag_0 = height_0 * width_0
            mag_1 = height_1 * width_1
            mag = np.sqrt(((mag_0 + mag_1)/2))

        if self.num_of_poles == 3:
            [az_l, el_l, height_l, width_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
            [az_r, el_r, height_r, width_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
            [az_c, el_c, height_c, width_c] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)

            mag_l = height_l * width_l
            mag_r = height_r * width_r
            mag_c = height_c * width_c

            if self.enter_right:
                az = ((az_c + az_r)/2)
                el = ((el_c + el_r)/2)
                mag = np.sqrt((mag_c + mag_r)/2)
            else:
                az = ((az_c + az_l)/2)
                el = ((el_c + el_l)/2)
                mag = np.sqrt((mag_c + mag_l)/2)
            
        return [az, el, mag, az_l, az_r, az_c]
    
    
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
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        self.target_class_print(dobj_nums)

        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[0]
        if len(dobj_nums) == 3:
            self.eval_startgate_poles(dobj_nums)
            dobj_num = self.pole_c

        watchdog_timer = Timeout(self.name + u"/Rétrace Watchdog".encode("utf-8"))

        # Enable the PID loops
        self.PID_enable()

        watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

        self.cuprint("servoing")

        self.cuprint(str(self.listener.check_new_dv(dobj_num, False)), warn=True)

        r = rospy.Rate(self.rate)
        print("") # overwritten by servoing status
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

                bearing_arr = self.ret_bearing(dobj_nums)

                az = bearing_arr[0]
                el = bearing_arr[1]
                mag = bearing_arr[2]
                az_l = bearing_arr[3]
                az_r = bearing_arr[4]
                az_c = bearing_arr[5]

                drive_setpoint = self.drive_client.get_standard_state()
                strafe_setpoint = self.strafe_client.get_standard_state()

                self.cuprint("CHECK POSITION: " + str(self.count) + " / " + str(self.count_target), print_prev_line=True)

                if self.enter_right:
                    pole_az = az_r
                else:
                    pole_az = az_l

                if abs(abs(pole_az) - abs(az_c)) < 0.08:
                    self.count += 1
                    if self.count > self.count_target:
                        self.cuprint("centered")
                        rospy.sleep(5)
                        break
                elif abs(abs(pole_az) - abs(az_c)) < 0.18:
                    if self.yaw_client.get_standard_state() < 0:
                        strafe_setpoint = self.strafe_client.get_standard_state() + 0.25
                    else:
                        strafe_setpoint = self.strafe_client.get_standard_state() - 0.25
                else:
                    self.count = 0
                    if self.yaw_client.get_standard_state() < 0:
                        strafe_setpoint = self.strafe_client.get_standard_state() + 0.75
                    else:
                        strafe_setpoint = self.strafe_client.get_standard_state() - 0.75

            # elif watchdog_timer.timed_out:
            #     self.PID_disable()
            #     self.cuprint("Retrace watchdog timed out.", warn=True)
            #     return "lost object"

            if userdata.timeout_obj.timed_out:
                watchdog_timer.timer.shutdown()
                userdata.outcome = "timed_out"
                return "not_found"
                
            self.PID_all_set_setpoint(drive_setpoint, strafe_setpoint, -1.5, az)
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.PID_disable()
        return "centered"



class Enter_With_Style(Approach):
    outcomes = ['finished', 'timed_out', 'lost_object']
    num_of_poles = 3

    def __init__(self, task_name, listener, clients):
        super(Enter_With_Style, self).__init__(task_name, listener, clients)

    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        self.cuprint("num_of_poles: " + str(self.num_of_poles), warn=True)

        # Find start_gate pole dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        self.target_class_print(dobj_nums)

        # DOBJECT NUMBER
        self.cuprint(str(dobj_nums), warn=True)
        dobj_num = dobj_nums[0]
        if len(dobj_nums) == 3:
            self.eval_startgate_poles(dobj_nums)
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

                bearing_arr = self.ret_bearing(dobj_nums)

                az = bearing_arr[0]
                el = bearing_arr[1]
                mag = bearing_arr[2]
                
                drive_setpoint = self.drive_client.get_standard_state()
                strafe_setpoint = self.strafe_client.get_standard_state()

                self.cuprint("mag: " + str(mag) + " - CHECK POSITION: " + str(self.count) + " / " + str(self.count_target), print_prev_line=True)

                if self.mag_target <= mag <= (1.33 * self.mag_target):
                    self.count += 1
                    drive_setpoint += 0.01
                    if self.count > self.count_target:
                        self.drive_client.set_setpoint(self.drive_client.get_standard_state() + 6.0, loop=False)
                        rospy.sleep(10)

                        radians_turned = 0
                        while radians_turned < 8*np.pi:
                            self.drive_client.set_setpoint(self.drive_client.get_standard_state())
                            yaw_setpoint = self.yaw_client.get_standard_state() + self.spin_carrot
                            radians_turned += self.spin_carrot
                            self.yaw_client.set_setpoint(yaw_setpoint, loop=False)
                            rospy.sleep(1)

                        self.yaw_client.set_setpoint(0, loop=False)
                        self.cuprint("finished")
                        break
                elif (self.mag_target - mag) > 0:
                    if self.count:
                        drive_setpoint += 0.3
                    else:
                        drive_setpoint += 0.6
                else:
                    self.count = 0
                    drive_setpoint -= 0.2

            elif watchdog_timer.timed_out:
                self.PID_disable()
                self.cuprint("Retrace watchdog timed out.", warn=True)
                return "lost object"

            if userdata.timeout_obj.timed_out:
                watchdog_timer.timer.shutdown()
                userdata.outcome = "timed_out"
                return "not_found"
                
            self.PID_all_set_setpoint(drive_setpoint, strafe_setpoint, -1.5, az)
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.PID_disable()
        return "finished"



# TODO
class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")