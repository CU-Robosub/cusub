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
import tf
import numpy as np

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
        self.approach_dist = rospy.get_param('tasks/jiangshi/approach_dist', 2.0)
        self.slay_dist = rospy.get_param('tasks/jiangshi/slay_dist', 0.5)

    def jiangshi_callback(self, msg):
        # Set the first pose and don't abort
        if self.jiangshi_pose == None:
            self.jiangshi_pose = msg
            return
        change_in_pose = self.get_distance(self.start_gate_pose.pose.position, msg.pose.position)
        if change_in_pose > self.replan_threshold:
            self.start_gate_pose = msg
            self.request_abort() # this will loop us back to execute

    def get_slay_path(self):
        """
        Calculate approach pose (also the backingup pose) and slaying pose

        Parameters
        ----------
        self.jiangshi_pose : PoseStamped
            Jiangshi's pose

        Returns
        -------
        approach_pose : Pose
        slay_pose : Pose
        """
        approach_pose = Pose()
        slay_pose = Pose()
        jiangshi_roll, jiangshi_pitch, jiangshi_yaw = tf.transformations.euler_from_quaternion(self.jiangshi_pose.pose.orientation)
        approach_pose.position.x = self.jiangshi_pose.pose.position.x + self.approach_dist * np.cos(jiangshi_yaw)
        approach_pose.position.y = self.jiangshi_pose.pose.position.y + self.approach_dist * np.sin(jiangshi_yaw)
        approach_pose.position.z = self.jiangshi_pose.pose.position.z
        approach_pose.orientation = tf.transformations.quaternion_from_euler(jiangshi_roll, jiangshi_pitch, jiangshi_yaw + np.pi)

        slay_pose.position.x = self.jiangshi_pose.pose.position.x - self.slay_dist * np.cos(jiangshi_yaw)
        slay_pose.position.y = self.jiangshi_pose.pose.position.y - self.slay_dist * np.sin(jiangshi_yaw)
        slay_pose.position.z = self.jiangshi_pose.pose.position.z
        slay_pose.orientation = approach_pose.orientation

        return approach_pose, slay_pose

    def execute(self, userdata):
        self.started = True
        self.clear_abort()
        approach_pose, slay_pose = self.get_slay_path()
        if self.go_to_pose(approach_pose) or \
            self.go_to_pose(slay_pose) or \
            self.go_to_pose(approach_pose, use_yaw=True):
            return 'aborted'
        else:
            return "success"
