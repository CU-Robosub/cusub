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

    def __init__(self):
        super(Jiangshi, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior_topic(), "cusub_cortex/mapper_out/jiangshi")
        self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Slay', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Slay', self.slay, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Slay(Objective):
    """
    Go to a point in front of the bouy, slay jiangshi backwards, backup
    """
    outcomes=['success', 'timed_out'] # Need to change a ton of shit here...

    def __init__(self):
        super(Slay, self).__init__(self.outcomes, "Slay")
        rospy.Subscriber("cusub_cortex/mapper_out/jiangshi", PoseStamped, self.jiangshi_callback)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)
        self.imu_axis = rospy.get_param("tasks/jiangshi/imu_axis")
        self.jump_thresh = rospy.get_param("tasks/jiangshi/jump_thresh")
        self.approach_dist = rospy.get_param('tasks/jiangshi/approach_dist', 2.0)
        self.slay_dist = rospy.get_param('tasks/jiangshi/slay_dist', 0.5)
        self.replan_threshold = rospy.get_param('tasks/jiangshi/replan_threshold', 0.5)
        self.use_buoys_yaw = rospy.get_param('tasks/jiangshi/use_buoys_yaw', True)
        self.jiangshi_pose = None
        self.monitor_imu = False

    def imu_callback(self, msg):
        if ( self.monitor_imu == False ) or self.replan_requested():
            return

        hit_detected = False

        if self.imu_axis == 'x':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.x < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.x > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        elif self.imu_axis == 'y':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.y < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.y > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        elif self.imu_axis == 'z':
            if (self.jump_thresh < 0) and ( msg.linear_acceleration.z < self.jump_thresh ):
                hit_detected = True
                self.request_replan()
            elif (self.jump_thresh > 0) and ( msg.linear_acceleration.z > self.jump_thresh ):
                hit_detected = True
                self.request_replan()
        else:
            rospy.logerr("Unrecognized imu axis: " + str(self.imu_axis))

        if hit_detected:
            rospy.loginfo_throttle(1, "Detected a buoy hit!")

    def jiangshi_callback(self, msg):
        # Set the first pose and don't replan
        if self.jiangshi_pose == None:
            self.jiangshi_pose = msg
            return
        change_in_pose = self.get_distance_xy(self.jiangshi_pose.pose.position, msg.pose.position)
        if (change_in_pose > self.replan_threshold) and not self.replan_requested():
            self.jiangshi_pose = msg
            self.request_replan()

    def get_slay_path(self):
        """
        Calculate approach pose (also the backingup pose) and slaying pose

        Parameters
        ----------
        self.jiangshi_pose : PoseStamped
            Jiangshi's pose
        self.use_buoys_yaw : bool
            Whether to calculate slay path and approach pose based on the buoys yaw
        self.approach_dist : float
            Meters from jiangshi to go upon approaching

        Returns
        -------
        approach_pose : Pose
        slay_pose : Pose
        """
        approach_pose = Pose()
        slay_pose = Pose()
        if self.use_buoys_yaw:    
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
        else: # just draw a line from sub to buoy and hit it
            approach_pose = self.get_pose_between(self.cur_pose, self.jiangshi_pose.pose, self.approach_dist) # inherited
            slay_pose = self.get_pose_behind(approach_pose, self.jiangshi_pose.pose, self.slay_dist)

        return approach_pose, slay_pose

    def execute(self, userdata):
        self.configure_darknet_cameras([1,1,0,0,1,0])
        while not rospy.is_shutdown():          # Loop until we find a good task pose
            approach_pose, slay_pose = self.get_slay_path()
            if self.go_to_pose(approach_pose, userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "done"
                else: # Loop, replan_requested() happened
                    pass
            else: # we've reached our pose!
                break
                    
        rospy.loginfo("...slaying buoy")
        rospy.sleep(2)
        self.monitor_imu = True
        self.go_to_pose(slay_pose, userdata.timeout_obj, move_mode="backup")
        self.monitor_imu = False
        self.go_to_pose(approach_pose, userdata.timeout_obj, replan_enabled=False)
        userdata.outcome = "success"
        return "success"