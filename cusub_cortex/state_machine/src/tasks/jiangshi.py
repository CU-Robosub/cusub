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
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import smach
import smach_ros
from sensor_msgs.msg import Imu

class Jiangshi(Task):
    name = "jiangshi"
    outcomes = ['task_success','task_aborted']

    def __init__(self):
        super(Jiangshi, self).__init__(self.outcomes)
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior_topic(), "cusub_cortex/mapper_out/jiangshi")
        self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Slay', 'not_found':'task_aborted'})
            smach.StateMachine.add('Slay', self.slay, transitions={'success':'task_success', 'aborted':'task_aborted'})

class Slay(Objective):
    """
    Go to a point in front of the bouy, slay jiangshi backwards, backup
    """
    outcomes=['success','aborted']

    def __init__(self):
        super(Slay, self).__init__(self.outcomes, "Slay")
        rospy.Subscriber("cusub_cortex/mapper_out/jiangshi", PoseStamped, self.jiangshi_callback)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)
        self.imu_axis = rospy.get_param("tasks/jiangshi/imu_axis")
        self.jump_thresh = rospy.get_param("tasks/jiangshi/jump_thresh")
        self.approach_dist = rospy.get_param('tasks/jiangshi/approach_dist', 2.0)
        self.slay_dist = rospy.get_param('tasks/jiangshi/slay_dist', 0.5)
        self.replan_threshold = rospy.get_param('tasks/jiangshi/replan_threshold', 0.5)
        self.jiangshi_pose = None
        self.monitor_imu = False
        self.jiangshi_jump = False

    def imu_callback(self, msg):
        if ( self.monitor_imu == False ) or self.abort_requested():
            return

        hit_detected = False

        if self.imu_axis == 'x':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.x < self.jump_thresh ):
                hit_detected = True
                self.request_abort()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.x > self.jump_thresh ):
                hit_detected = True
                self.request_abort()
        elif self.imu_axis == 'y':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.y < self.jump_thresh ):
                hit_detected = True
                self.request_abort()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.y > self.jump_thresh ):
                hit_detected = True
                self.request_abort()
        elif self.imu_axis == 'z':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.z < self.jump_thresh ):
                hit_detected = True
                self.request_abort()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.z > self.jump_thresh ):
                hit_detected = True
                self.request_abort()
        else:
            rospy.logerr("Unrecognized imu axis: " + str(self.imu_axis))

        if hit_detected:
            rospy.loginfo_throttle(1, "Detected a buoy hit!")

    def jiangshi_callback(self, msg):
        # Set the first pose and don't abort
        if self.jiangshi_pose == None:
            self.jiangshi_pose = msg
            return
        change_in_pose = self.get_distance_xy(self.jiangshi_pose.pose.position, msg.pose.position)
        if (change_in_pose > self.replan_threshold) and not self.jiangshi_jump:
            self.jiangshi_pose = msg
            self.cancel_wp_goal() # this will loop us back to execute
            self.jiangshi_jump = True

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
        quat = [self.jiangshi_pose.pose.orientation.x, self.jiangshi_pose.pose.orientation.y,self.jiangshi_pose.pose.orientation.z,self.jiangshi_pose.pose.orientation.w]
        jiangshi_roll, jiangshi_pitch, jiangshi_yaw = tf.transformations.euler_from_quaternion(quat)
        approach_pose.position.x = self.jiangshi_pose.pose.position.x + self.approach_dist * np.cos(jiangshi_yaw)
        approach_pose.position.y = self.jiangshi_pose.pose.position.y + self.approach_dist * np.sin(jiangshi_yaw)
        approach_pose.position.z = self.jiangshi_pose.pose.position.z
        goal_quat = tf.transformations.quaternion_from_euler(jiangshi_roll, jiangshi_pitch, jiangshi_yaw)
        approach_pose.orientation.x = goal_quat[0]
        approach_pose.orientation.x = goal_quat[1]
        approach_pose.orientation.x = goal_quat[2]
        approach_pose.orientation.x = goal_quat[3]

        slay_pose.position.x = self.jiangshi_pose.pose.position.x - self.slay_dist * np.cos(jiangshi_yaw)
        slay_pose.position.y = self.jiangshi_pose.pose.position.y - self.slay_dist * np.sin(jiangshi_yaw)
        slay_pose.position.z = self.jiangshi_pose.pose.position.z
        slay_pose.orientation = approach_pose.orientation

        return approach_pose, slay_pose

    def execute(self, userdata):
        self.clear_abort()
        self.configure_darknet_cameras([1,1,0,0,1,0])
        while not rospy.is_shutdown():          # Loop until we find a good SG pose
            approach_pose, slay_pose = self.get_slay_path()
            if self.go_to_pose(approach_pose):
                if self.jiangshi_jump:  # Loop again and replan Jiangshi
                    rospy.loginfo("...replanning")
                    self.jiangshi_jump = False
                else:
                    return "aborted"
            else:
                break
                    
        rospy.loginfo("---slaying buoy")
        rospy.sleep(2)
        self.monitor_imu = True
        self.go_to_pose(slay_pose, move_mode="backup")
        self.monitor_imu = False
        self.clear_abort()
        self.go_to_pose(approach_pose)
        return "success"