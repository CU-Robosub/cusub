#!/usr/bin/env python
from __future__ import division
"""
Manager State
"""
import smach
import smach_ros
from tasks.task import Timeout
import rospy

# Possible task statuses:
# queued, active, success, timedout, not_found

class Manager(smach.State):

    def __init__(self, task_names, mission_end_outcome='mission_success'):
        rospy.loginfo("Loading manager obj")
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

        # service to get the next available class

    def execute(self, userdata):
        rospy.loginfo("Executing Manager")
        if userdata.previous_outcome == "starting":
            pass
        elif userdata.previous_outcome == "success":
            self.success()
        else:
            response_param = "tasks/"+self.queued_tasks[0]+"/"+userdata.previous_outcome
            prev_outcome_response = rospy.get_param(response_param)
            if prev_outcome_response == "skip":
                self.skip()
            elif prev_outcome_response == "later":
                self.later()
            else:
                raise(Exception("Manager doesn't recognize '" + prev_outcome_response + "' from param '" + response_param + "'"))

        if len(self.queued_tasks) == 0:
            return self.mission_end_outcome
        secs = rospy.get_param("tasks/"+self.queued_tasks[0]+"/timeout_secs")
        self.timeout_obj.set_new_time(secs)
        return self.queued_tasks[0]

    def success(self):
        rospy.loginfo("Success")
        self.queued_tasks.pop(0)
    def skip(self):
        rospy.loginfo("Skipping")
        self.queued_tasks.pop(0)
        pass
    def later(self):
        rospy.loginfo("Latering")
        self.queued_tasks.pop(0)
        pass
        
    # get next task service, should just be the 2nd index in the array

