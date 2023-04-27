#!/usr/bin/env python
from __future__ import division
"""
Triangule Buoy Task, attempts to slay a particular class on the triangle buoy
Objectives:
- Search
- Slay
---> Go to approach point
---> Slay vampire
---> Backup
"""
import rospy
from tasks.task import Task, Objective
from tasks.search import Search
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
import smach
import smach_ros
from perception_control.msg import OrbitBuoyAction, OrbitBuoyGoal
import actionlib
import tf

class Triangle_Buoy(Task):
    name = "triangle_buoy"
    outcomes = ['task_success','task_aborted']

    def __init__(self):
        super(Triangle_Buoy, self).__init__(self.outcomes)
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior(), "cusub_cortex/mapper_out/triangle_buoy")
        self.slay = Slay()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Slay', 'not_found':'task_aborted'})
            smach.StateMachine.add('Slay', self.slay, transitions={'success':'task_success', 'aborted':'Slay'})

class Slay(Objective):
    """
        Go to a point in front of the bouy, make actionlib orbit request, slay dat buoy
    """
    outcomes=['success','aborted']

    def __init__(self):
        super(Slay, self).__init__(self.outcomes, "Slay")
        self.approaching = False
        self.imu_axis = rospy.get_param("tasks/triangle_buoy/imu_axis")
        self.jump_thresh = rospy.get_param("tasks/triangle_buoy/jump_thresh")
        self.approach_dist = rospy.get_param('tasks/triangle_buoy/approach_dist', 2.0)
        self.slay_dist = rospy.get_param('tasks/triangle_buoy/slay_dist', 0.5)
        self.replan_threshold = rospy.get_param('tasks/triangle_buoy/replan_threshold', 0.5)
        self.slay_class = rospy.get_param("tasks/triangle_buoy/slay_class")
        if rospy.get_param("tasks/triangle_buoy/orbital_direction") == "left":
            self.orbit_left = True
            rospy.loginfo("---orbiting traingle buoy LEFT")
        else:
            self.orbit_left = False
            rospy.loginfo("---orbiting triangle buoy RIGHT")
        self.strafe_setpoint = rospy.get_param("tasks/triangle_buoy/strafe_setpoint")
        self.number_solo_frames = rospy.get_param("tasks/triangle_buoy/number_solo_frames")
        self.triangle_buoy_pose = None
        self.monitor_imu = False
        rospy.logwarn("---waiting for orbit_buoy service")
        self.client = actionlib.SimpleActionClient('orbit_buoy', OrbitBuoyAction)
        self.client.wait_for_server()
        rospy.loginfo("---found orbit_buoy service")
        rospy.Subscriber("cusub_cortex/mapper_out/triangle_buoy", PoseStamped, self.triangle_buoy_pose_callback)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)

    def triangle_buoy_pose_callback(self, msg):
        # Set the first pose and don't abort
        if self.triangle_buoy_pose == None:
            self.triangle_buoy_pose = msg
            return
        change_in_pose = self.get_distance(self.triangle_buoy_pose.pose.position, msg.pose.position)
        if (change_in_pose > self.replan_threshold) and self.approaching:
            rospy.logwarn_throttle(1,"Triangle Buoy Pose jump.")
            print(self.triangle_buoy_pose.pose.position)
            print(msg.pose.position)
            self.triangle_buoy_pose = msg
            self.request_abort() # this will loop us back to execute

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

    def execute(self, userdata):
        """
        Approach bouy
        Make orbit actionlib req, success case is class detected
        Task code cancels the req
        proceed to slaying and backing up
        """
        self.approaching = True
        self.clear_abort()
        self.configure_darknet_cameras([1,0,0,0,0,0])
        approach_pose = self.get_pose_between(self.cur_pose, self.triangle_buoy_pose.pose, self.approach_dist) # inherited
        if self.go_to_pose(approach_pose):
            return "aborted"

        self.approaching = False # Only abort b/c of pose jumps when we're approaching

        # Create Orbit Goal
        goal = OrbitBuoyGoal()
        goal.target_class = self.slay_class
        goal.buoy_pose = self.triangle_buoy_pose.pose
        goal.strafe_setpoint = self.strafe_setpoint
        goal.number_solo_frames = self.number_solo_frames
        goal.orbit_left = self.orbit_left
        goal.orbit_radius = self.approach_dist

        rospy.loginfo("Sending Goal to Buoy Orbitter")
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(180))

        # Slay a buoy either way...

        preslay_pose = self.cur_pose
        slay_pose = self.get_pose_behind(preslay_pose, self.triangle_buoy_pose.pose, self.slay_dist)

        self.monitor_imu = True
        self.go_to_pose(slay_pose, move_mode="backup")
        self.monitor_imu = False
        self.clear_abort()
        self.go_to_pose(preslay_pose)

        # Now slay the buoy
        return "success"