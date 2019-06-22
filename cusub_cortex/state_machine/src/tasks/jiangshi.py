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
    
    """
    outcomes=['success','aborted']

    def __init__(self):
        super(Approach, self).__init__(self.outcomes, "Approach")
        rospy.Subscriber("cusub_cortex/mapper_out/start_gate", PoseStamped, self.start_gate_callback)
        self.started = False

    def start_gate_callback(self, msg):
        # Set the first pose and don't abort the Attack Objective
        if self.start_gate_pose == None:
            self.start_gate_pose = msg
            return

        changeInPose = self.getDistance(self.start_gate_pose.pose.position, msg.pose.position)

        if changeInPose > self.replan_threshold:
            self.start_gate_pose = msg
            self.request_abort() # this will loop us back to execute

    def execute(self, userdata):
        self.started = True
        rospy.loginfo("Executing Attack")
        self.clear_abort()

        target_pose = self.adjust_gate_pose(
            self.cur_pose, \
            self.start_gate_pose.pose, \
            self.dist_behind, \
            self.small_leg_left_side, \
            self.leg_adjustment_meters)
        
        if self.goToPose(target_pose):
            return 'aborted'
        return "success"

class Slay(Objective):
    """

    """