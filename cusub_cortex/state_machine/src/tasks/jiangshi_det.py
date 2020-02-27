#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function
"""
Jiangshi Buoy Task, attempts to bump into the jiangshi buoy
Objectives:
- Search
- Approach
- Slay
- Back Up
"""
from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import tf
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Time
from detection_listener.listener import DetectionListener
from waypoint_navigator.srv import ToggleControl
import numpy as np
from cusub_print.cuprint import bcolors
import sys

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
        self.slay = Slay(self.name)
        self.retrace = Retrace(self.name, self.listener)
        self.backup = Backup(self.name)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Slay', self.slay, transitions={'slayed':'Backup', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Backup', self.backup, transitions={'backed_up':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'Slay', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']

    target_class_id = "vampire_cute"

    def __init__(self, task_name, listener):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive", "cusub_common/motor_controllers/mag_pid/")
        self.depth_client = PIDClient(name, "depth", "cusub_common/motor_controllers/elev_pid/")

        self.retrace_timeout = rospy.get_param("tasks/" + task_name.lower() + "/retrace_timeout", 2)
        seconds = rospy.get_param("tasks/" + task_name.lower() + "/seconds_in_position", 2)
        self.rate = rospy.get_param("tasks/" + task_name.lower() + "/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0
        
        self.mag_target = rospy.get_param("tasks/" + task_name.lower() + "/mag_target")
        self.elev_target = rospy.get_param("tasks/" + task_name.lower() + "/elev_target")
        self.elev_thresh = rospy.get_param("tasks/" + task_name.lower() + "/elev_thresh")
    
    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Find vampire_cute's dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target_class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.cuprint("selecting the first", warn=True)
        elif not dobj_nums: # Chck if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        dobj_num = dobj_nums[0]
        self.cuprint("located " + self.target_class_id + "'s dobject num: " + bcolors.HEADER + str(dobj_num) + bcolors.ENDC)
        
        # Start a watch dog timer on detections
        watchdog_timer = Timeout(self.name)

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(self.elev_target)

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)
                [az, el, mag] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                self.yaw_client.set_setpoint(az, loop=False)
                self.drive_client.set_state(mag)
                self.depth_client.set_state(el * (30000/np.sqrt(mag))) # normalize the elevation bearing using the magnitude

                if self.check_in_position(az, el, mag):
                    self.count += 1
                    if self.count > self.count_target:
                        self.cuprint("in position")
                        break
                else:
                    self.count = 0
            elif watchdog_timer.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                return "lost_object"

            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"

            self.drive_client.set_setpoint(self.mag_target, loop=False)
            self.depth_client.set_setpoint(self.elev_target, loop=False)
            r.sleep()

        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "in_position"

    def check_in_position(self, az, el, mag):
        az_reached, el_reached, mag_reached = True, False, False
        if (self.mag_target - mag) < 0.1*self.mag_target:
            mag_reached = True
        if (self.elev_target - el) < self.elev_thresh:
            el_reached = True
        return az_reached and el_reached and mag_reached

class Slay(Objective):
    outcomes = ['slayed', 'timed_out']

    def __init__(self, task_name):
        name = task_name + "/Slay"
        super(Slay, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/" + task_name.lower() + "/slay_carrot_dist")

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

class Backup(Objective):
    outcomes = ['backed_up','timed_out']

    def __init__(self, task_name):
        name = task_name + "/Backup"
        super(Backup, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/" + task_name.lower() + "/slay_carrot_dist")

    def execute(self, userdata):
        self.cuprint("executing")
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.enable()
        self.drive_client.set_setpoint(cur_state - self.carrot_dist)
        rospy.sleep(10)
        self.toggle_waypoint_control(False)
        self.drive_client.disable()
        self.cuprint("backed up")
        return "backed_up"


class Retrace(Objective):
    outcomes = ['found','not_found']

    target_class_id = "vampire_cute"

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + u"/Rétrace".encode("utf-8"))
        self.listener = listener
        self.retrace_hit_cnt = rospy.get_param("tasks/" + task_name.lower() + "/retrace_hit_count")
        self.retrace_back_in_time = rospy.get_param("tasks/" + task_name.lower() + "/retrace_back_in_time")

    def execute(self, userdata):
        self.toggle_waypoint_control(False)
        self.cuprint("executing")
        dobj_nums = self.listener.query_class(self.target_class_id)
        if not len(dobj_nums): #shouldn't be possible
            # do some timeout?
            userdata.outcome = "timed_out"
            return "not_found"

        #This is an assumption that we only have one dobj, which we *should*
        dobj_num = dobj_nums[0]

        # loop variables
        count = 0
        retraced_steps = 1
        first_bc = self.listener[dobj_num][-1]

        backup_time = first_bc.camera_header.stamp - rospy.Duration(self.retrace_back_in_time)
        
        breadcrumb_dvs = self.listener[dobj_num].get_dvectors_since_time(backup_time)

        len_dvec = len(breadcrumb_dvs)
        self.cuprint("breadcrumbs to follow since " + str(self.retrace_back_in_time) + " seconds ago:" + str(len_dvec))
        last_sub_pt = breadcrumb_dvs[len_dvec-retraced_steps].sub_pt 
        last_pose = Pose(last_sub_pt, self.cur_pose.orientation)

        # Start Retrace: Set first waypoint
        self.go_to_pose_non_blocking(last_pose)
        # necessary for same-line printing
        print("")
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num) and count < self.retrace_hit_cnt: 
                #found object again
                count += 1
            elif count >= self.retrace_hit_cnt:
                self.cancel_way_client_goal()
                return "found"
            else:
                dist = self.get_distance(self.cur_pose.position, last_pose.position)
                self.cuprint("Breadcrumb #"+ bcolors.HEADER + str( retraced_steps)+ bcolors.ENDC+ " distance: " + bcolors.OKBLUE + str(dist) + bcolors.ENDC, print_prev_line=True)
                #self.cuprint("distance to goal: "+ str())
                if self.check_reached_pose(last_pose, 0.7):
                    self.cancel_way_client_goal()
                    retraced_steps += 1
                    if (retraced_steps > len_dvec):
                        #means we retraces all our steps. We're lost.
                        return "not_found"
                    # Goto Last dvector
                    # get last dvector sub_pt
                    last_sub_pt = breadcrumb_dvs[len_dvec-retraced_steps].sub_pt 
            
                    last_pose = Pose(last_sub_pt, self.cur_pose.orientation)
                    #set waypoint to this point. Give some distance to account for error in bearing and sub_pt
                    # Set waypoint
                    self.go_to_pose_non_blocking(last_pose, log_print=False)

            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"

        #clean up if we are killed
        return "not_found"
        


