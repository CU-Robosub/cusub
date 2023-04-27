#!/usr/bin/env python
from __future__ import division
"""
Manager State
Manages state transitions between tasks
Reads in the state transitions from the mission_config parameters
"""
import smach
import smach_ros
from tasks.task import Timeout
import rospy
from state_machine.msg import TaskStatus
from state_machine.srv import *
from cusub_print.cuprint import CUPrint

class Manager(smach.State):
    name = "Manager"

    def __init__(self, task_names, mission_end_outcome='mission_success'):
        self.cuprint = CUPrint(self.name)
        self.cuprint("initializing")
        self.queued_tasks = task_names
        self.mission_end_outcome = mission_end_outcome
        outcomes = self.queued_tasks + [mission_end_outcome]
        smach.State.__init__(self,
            outcomes=outcomes, \
            input_keys=['previous_outcome'], \
            output_keys=['timeout_obj'])

        self.timeout_obj = Timeout()
        self.tasks_status = {}
        for task in self.queued_tasks:
            self.tasks_status[task] = 'queued'

        self.pub_seq = 0
        self.pub = rospy.Publisher("cusub_cortex/state_machine/task_status", TaskStatus, queue_size=1)
        rate = rospy.get_param("manager_task_status_pub_rate")
        rospy.Timer(rospy.Duration(1 / rate), self.publish_task_states_callback)

        # service to get the next available class
        rospy.Service('cusub_cortex/state_machine/get_next_task', GetNextTask, self.handle_get_next_task)

    def execute(self, userdata):
        if userdata.previous_outcome == "starting":
            pass
        elif userdata.previous_outcome == "success":
            self.tasks_status[self.queued_tasks[0]] = "success"
            self.move_on()
        else:
            self.tasks_status[self.queued_tasks[0]] = userdata.previous_outcome
            response_param = "tasks/"+self.queued_tasks[0]+"/manager_info/outcome_"+userdata.previous_outcome
            prev_outcome_response = rospy.get_param(response_param)
            if prev_outcome_response == "skip":
                self.move_on()
            elif prev_outcome_response == "later":
                self.later()
            else:
                raise(Exception("Manager doesn't recognize '" + prev_outcome_response + "' from param '" + response_param + "'"))

        if len(self.queued_tasks) == 0:
            return self.mission_end_outcome
        self.tasks_status[self.queued_tasks[0]] = "active"
        secs = rospy.get_param("tasks/"+self.queued_tasks[0]+"/manager_info/timeout_secs")
        self.timeout_obj.set_new_time(secs)
        userdata.timeout_obj = self.timeout_obj
        return self.queued_tasks[0]

    def move_on(self):
        self.queued_tasks.pop(0)
    def later(self):
        rospy.loginfo("Latering")
        task = self.queued_tasks.pop(0)
        self.queued_tasks.append(task)
    
    def publish_task_states_callback(self, msg):
        ts = TaskStatus()
        for task in self.tasks_status.keys():
            ts.task_statuses.append(task)
            ts.task_statuses.append(self.tasks_status[task])
        ts.header.seq = self.pub_seq; self.pub_seq += 1
        ts.header.stamp = rospy.Time.now()
        self.pub.publish(ts)

    def handle_get_next_task(self, req):
        if len(self.queued_tasks) < 2:
            return GetNextTaskResponse("mission_complete")
        else:
            return GetNextTaskResponse(self.queued_tasks[1])
