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

        self.center_strafe = Center_Strafe(self.name, self.listener, clients)
        self.penetrate_with_style = Penetrate_With_Style(self.name, self.listener, clients)

        self.retrace = Retrace(self.name, self.listener)



    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach', self.approach, transitions={'found_pole':'Approach_2_Poles', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_2_Poles', self.approach_2_poles, transitions={'found_3rd_pole':'Approach_3_Poles', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Approach_3_Poles', self.approach_3_poles, transitions={'in_position':'Center_Strafe', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Center_Strafe', self.center_strafe, transitions={'centered':'Penetrate_With_Style', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

            smach.StateMachine.add('Penetrate_With_Style', self.penetrate_with_style, transitions={'finished':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
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

        self.mag_target = rospy.get_param("tasks/start_gate/center_pole_target")


    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Initialize Startgate Pole values
        self.pole_l = self.pole_r = self.pole_c = -1

        self.cuprint("num_of_poles: " + str(self.num_of_poles), warn=True)

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
        self.yaw_client.enable()

        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(-1.5)

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
                
                self.drive_setpoint = self.drive_client.get_standard_state()

                self.cuprint("[MAGNITUDE: " + str(mag) + "] - " + "[AZ: " + str(az) + "] - " +  "[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]", print_prev_line=True)

                if abs(self.mag_target - mag) < (0.5 * self.mag_target):
                    self.count += 1
                    self.drive_setpoint += 0.15
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        self.yaw_client.set_setpoint(0, loop=False)
                        self.drive_client.set_setpoint(self.drive_setpoint, loop=False)
                        rospy.sleep(5)
                        break
                elif (self.mag_target - mag) > 0:
                    if self.count:
                        self.drive_setpoint += 0.2
                    else:
                        self.drive_setpoint += 0.6
                else:
                    self.count = 0
                    self.drive_setpoint -= 0.2

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
                
            self.drive_client.set_setpoint(self.drive_setpoint, loop=False)
            self.depth_client.set_setpoint(-1.5, loop=False)
            self.yaw_client.set_setpoint(az, loop=False)
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.drive_client.disable()
        self.strafe_client.disable()
        self.depth_client.disable()
        self.yaw_client.disable()
        return "found_pole"


    def ret_bearing(self, dobj_nums):
        [az_l, el_l, height_l, width_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
        [az_r, el_r, height_r, width_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
        [az_c, el_c, height_c, width_c] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)
        mag_l = height_l * width_l
        mag_r = height_r * width_r
        mag_c = height_c * width_c
        az = az_c
        el = el_c
        mag = np.sqrt(mag_c)

        return [az, el, mag]
    
    
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



class Center_Strafe(Approach):
    outcomes = ['centered', 'timed_out', 'lost_object']

    def __init__(self, task_name, listener):
        name = task_name + "/Center_Strafe"
        super(Approach, self).__init__(self.outcomes, name)
        self.target_class_id = "start_gate_pole"
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
        super(Center_Strafe, self).eval_startgate_poles(dobj_nums)
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
                [az_l, el_l, height_l, width_l] = self.listener.get_avg_bearing(self.pole_l, num_dv=5)
                [az_r, el_r, height_r, width_r] = self.listener.get_avg_bearing(self.pole_r, num_dv=5)
                [az_c, el_c, height_c, width_c] = self.listener.get_avg_bearing(self.pole_c, num_dv=5)

                mag_l = height_l * width_l
                mag_r = height_r * width_r
                mag_c = height_c * width_c
                
                az_diff = 0
                az_of_interest = 0
                if self.orbit_right:
                    az_diff = abs(abs(az_r) - abs(az_c))
                    az_of_interest = az_r
                else:
                    az_diff = abs(abs(az_l) - abs(az_c))
                    az_of_interest = az_l

                self.cuprint("AZ: [L] " + str(az_l) + " - [C] " + str(az_c) + " - [R] " + str(az_r))

                 # Adjust strafe with carrot on current strafe
                strafe_msg = Float64()
                strafe_set = 0.0 if self.strafe_state is None else self.strafe_state


                if (az_c < 0):
                    self.cuprint("LEFT SIDE OF GATE")
                    self.cuprint("az_diff: " + str(az_diff))
                    self.cuprint("az_of_interest: " + str(az_of_interest))
                    # if 0 <= az_diff <= 0.08:
                    #     self.cuprint("[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    #     if self.count == 1:
                    #         self.in_position_state = self.drive_client.get_standard_state()
                    #     if self.in_position_state is not None:
                    #         self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    #     self.count += 1
                    #     # Stop Sub
                    #     if self.count > self.count_target:
                    #         self.cuprint("centered")
                    #         self.yaw_client.set_setpoint(0, loop=False)
                    #         self.drive_client.increment_setpoint(0.15, loop=False)
                    #         rospy.sleep(5)
                    #         break
                    if self.orbit_right:
                        if (az_diff < 0.1) or (az_of_interest < 1.0):
                            self.cuprint("strafing right")
                            strafe_msg.data = strafe_set + self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        elif az_diff > 0.3:
                            self.cuprint("strafing left")
                            strafe_msg.data = strafe_set - self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        else:
                            # Not within threshold
                            self.count = 0
                    else:
                        if (az_diff < 0.1) and (az_of_interest < 0.0):
                            self.cuprint("strafing right")
                            strafe_msg.data = strafe_set + self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        elif az_diff > 0.1:
                            self.cuprint("strafing left")
                            strafe_msg.data = strafe_set - self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        else:
                            # Not within threshold
                            self.count = 0
                elif (az_c > 0):
                    self.cuprint("RIGHT SIDE OF GATE")
                    self.cuprint("az_diff: " + str(az_diff))
                    self.cuprint("az_of_interest: " + str(az_of_interest))
                    # if 0 <= az_diff <= 0.08:
                    #     self.cuprint("[CHECK POSITION: " + str(self.count) + " / " + str(self.count_target) + "]")
                    #     if self.count == 1:
                    #         self.in_position_state = self.drive_client.get_standard_state()
                    #     if self.in_position_state is not None:
                    #         self.drive_client.set_setpoint(self.in_position_state, loop=False)
                    #     self.count += 1
                    #     # Stop Sub
                    #     if self.count > self.count_target:
                    #         self.cuprint("centered")
                    #         self.yaw_client.set_setpoint(0, loop=False)
                    #         self.drive_client.increment_setpoint(0.15, loop=False)
                    #         rospy.sleep(5)
                    #         break
                    if self.orbit_right:
                        if (az_diff < 0.1) or (az_of_interest > 1.0):
                            strafe_msg.data = strafe_set + self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        elif az_diff > 0.3:
                            strafe_msg.data = strafe_set - self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        else:
                            # Not within threshold
                            self.count = 0
                    else:
                        if az_diff < 0.1:
                            strafe_msg.data = strafe_set + self.strafe_carrot
                            self.strafe_pub.publish(strafe_msg)
                        elif az_diff > 0.1:
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
                
            self.depth_client.set_setpoint(-1.5, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "centered"


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
        self.depth_client.set_setpoint(-1.5, loop=False)

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

# TODO
class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")