#!/usr/bin/env python
from __future__ import division
"""
Jiangshi Buoy Task, attempts to slay Jiangshi
Objectives:
- Search
- Approach
- Slay

"""
from tasks.task import Task, Objective
from tasks.search import Search

class Jiangshi(Task):
    name = "jaingshi"
    outcomes = ['task_success','task_aborted']

    def __init__(self):
        super(Jiangshi, self).__init__(self.outcomes)
        self.initObjectives()
        self.linkObjectives()

    def initObjectives(self):
        self.search = Search(self.getPrior(), "cusub_cortex/mapper_out/jiangshi")
        # self.approach = Approach()
        self.slay = Slay()

    def linkObjectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Attack', 'not_found':'Search'})

class Approach(Objective):
    """
    Go to a point in front of the bouy
    """
    outcomes=['success','aborted']

    def __init__(self):
        super(Approach, self).__init__(self.outcomes, "Approach")
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
        rospy.loginfo("Executing Approach")
        self.clear_abort()

        target_pose = self.adjust_gate_pose(
            self.cur_pose, \
            self.start_gate_pose.pose, \
            self.dist_behind, \
            self.small_leg_left_side, \
            self.leg_adjustment_meters)
        
        if self.go_to_pose(target_pose):
            return 'aborted'
        return "success"

class Slay(Objective):
    """
    Hit buoy and backup
    """
    pass