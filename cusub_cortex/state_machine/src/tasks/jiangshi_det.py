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
        self.revisit = Revisit(self.name, self.listener)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Revisit', self.revisit, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})                
            smach.StateMachine.add('Approach', self.approach, transitions={'success':'manager', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['success','timed_out', 'lost_object']

    target_class_id = "vampire_cute"

    def __init__(self, task_name, listener):
        super(Approach, self).__init__(self.outcomes, task_name + "/Approach")
        self.listener = listener
        self.yaw_client = PIDClient("yaw")
        self.drive_client = PIDClient("drive", "cusub_common/motor_controllers/mag_pid/")

        self.mag_target = 152600
    
    def execute(self, userdata):
        self.smprint("executing")

        # Find vampire_cute's dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1:
            self.smprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.smprint("selecting the first", warn=True)
        elif not dobj_nums:
            self.smprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        dobj_num = dobj_nums[0]
        self.smprint("located " + self.target_class_id + "'s dobject num: " + str(dobj_num))
        
        # TODO start a watch dog timer on detections

        # Begin looping
        # --- if timeout --> stop PID's and exit state
        # --- if new dv available
        # --- grab most recent dvector
        # --- set state/setpoint for the 3 PID loops
        # --- determine success conditions: check magnitude + elevation

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.drive_client.set_setpoint(self.mag_target)
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az, el, mag] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                self.yaw_client.set_setpoint(az, loop=False)
                self.drive_client.set_state(mag)
                self.listener.clear_new_dv_flag(dobj_num)
            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"
            self.drive_client.set_setpoint(self.mag_target, loop=False)
            r.sleep()
        userdata.outcome = "success"
        return "success"

    def check_in_position(self, az, el, mag):
        pass

class Slay(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Slay, self).__init__(self.outcomes, task_name + "/Slay")


class Backup(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Backup, self).__init__(self.outcomes, task_name + "/Backup")

class Revisit(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Revisit, self).__init__(self.outcomes, task_name + "/Revisit")