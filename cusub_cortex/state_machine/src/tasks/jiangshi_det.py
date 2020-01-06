#!/usr/bin/env python
from __future__ import division
"""
Jiangshi Buoy Task, attempts to bump into the jiangshi buoy
Objectives:
- Search
- Approach
- Slay
- Back Up
"""
from tasks.task import Task, Objective
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np

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
        self.revisit = Revisit(self.name, self.listener)
        self.backup = Backup(self.name)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Revisit', self.revisit, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Slay', self.slay, transitions={'slayed':'Backup', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Backup', self.backup, transitions={'backed_up':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'Slay', 'timed_out':'manager', 'lost_object':'Revisit'}, \
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

        seconds = rospy.get_param("tasks/jiangshi/seconds_in_position")
        self.rate = rospy.get_param("tasks/jiangshi/new_dv_check_rate")
        self.count_target = seconds * self.rate
        self.count = 0
        
        self.mag_target = rospy.get_param("tasks/jiangshi/mag_target")
        self.elev_target = rospy.get_param("tasks/jiangshi/elev_target")
        self.elev_thresh = rospy.get_param("tasks/jiangshi/elev_thresh")
    
    def execute(self, userdata):
        self.smprint("executing")

        # Find vampire_cute's dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target_class
            self.smprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.smprint("selecting the first", warn=True)
        elif not dobj_nums: # Chck if target class is not present (shouldn't be possible)
            self.smprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        dobj_num = dobj_nums[0]
        self.smprint("located " + self.target_class_id + "'s dobject num: " + str(dobj_num))
        
        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()
        self.drive_client.set_setpoint(self.mag_target)
        self.depth_client.set_setpoint(self.elev_target)

        self.smprint("servoing")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az, el, mag] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                self.yaw_client.set_setpoint(az, loop=False)
                self.drive_client.set_state(mag)
                self.depth_client.set_state(el * (1000/np.sqrt(mag))) # normalize the elevation bearing using the magnitude
                self.listener.clear_new_dv_flag(dobj_num)

                if self.check_in_position(az, el, mag):
                    self.count += 1
                    if self.count > self.count_target:
                        self.smprint("in position")
                        break
                else:
                    self.count = 0
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
        self.carrot_dist = rospy.get_param("tasks/jiangshi/slay_carrot_dist")

    def execute(self, userdata):
        self.smprint("executing")
        self.wayToggle(False)
        self.drive_client.enable()
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.set_setpoint(cur_state + self.carrot_dist)
        rospy.sleep(10)
        self.smprint("slayed")
        self.drive_client.disable()
        return "slayed"

class Backup(Objective):
    outcomes = ['backed_up','timed_out']

    def __init__(self, task_name):
        name = task_name + "/Backup"
        super(Backup, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/jiangshi/slay_carrot_dist")

    def execute(self, userdata):
        self.smprint("executing")
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.enable()
        self.drive_client.set_setpoint(cur_state - self.slay_carrot_dist)
        rospy.sleep(10)
        self.wayToggle(True)
        self.drive_client.disable()
        self.smprint("backed up")
        return "backed_up"


# TODO
class Revisit(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Revisit, self).__init__(self.outcomes, task_name + "/Revisit")