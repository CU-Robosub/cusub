#!/usr/bin/env python
from __future__ import division
"""
Jiangshi Buoy Task, attempts to slay Jiangshi
Objectives:
- Search
- Slay
---> Go to approach point
---> Slay vampire
---> Backup
"""
from tasks.task import Task, Objective
from tasks.search import Search

class Jiangshi(Task):
    name = "jaingshi"
    outcomes = ['task_success','task_aborted']

    def __init__(self):
        super(Jiangshi, self).__init__(self.outcomes)
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior(), "cusub_cortex/mapper_out/jiangshi")
        self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Attack', 'not_found':'Search'})
            smach.StateMachine.add('Slay', self.search, transitions={'success':'task_success', 'aborted':'Slay'})

class Slay(Objective):
    """
    Go to a point in front of the bouy
    """
    outcomes=['success','aborted']

    def __init__(self):
        super(Approach, self).__init__(self.outcomes, "Slay")
        rospy.Subscriber("cusub_cortex/mapper_out/jiangshi", PoseStamped, self.jiangshi_callback)
        self.started = False

    def jiangshi_callback(self, msg):
        # Set the first pose and don't abort
        if self.jiangshi_pose == None:
            self.jiangshi_pose = msg
            return
        change_in_pose = self.get_distance(self.start_gate_pose.pose.position, msg.pose.position)
        if change_in_pose > self.replan_threshold:
            self.start_gate_pose = msg
            self.request_abort() # this will loop us back to execute

    def execute(self, userdata):
        self.started = True
        self.clear_abort()

        # Calculate the line according to the vector of the pose
        # go to x meters out in front of pose
        # Slay bouy by going y meters behind buoy
        # Backup to x meters using STRAFE MODE
        
        if self.go_to_pose(target_pose):
            return 'aborted'
        return "success"