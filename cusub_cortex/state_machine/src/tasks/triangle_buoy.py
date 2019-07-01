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
        rospy.Subscriber("cusub_cortex/mapper_out/triangle_buoy", PoseStamped, self.triangle_buoy_pose_callback)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)
        self.started = False
        self.imu_axis = rospy.get_param("tasks/triangle_buoy/imu_axis")
        self.jump_thresh = rospy.get_param("tasks/triangle_buoy/jump_thresh")
        self.approach_dist = rospy.get_param('tasks/triangle_buoy/approach_dist', 2.0)
        self.slay_dist = rospy.get_param('tasks/triangle_buoy/slay_dist', 0.5)
        self.replan_threshold = rospy.get_param('tasks/triangle_buoy/replan_threshold', 0.5)
        self.triangle_buoy_pose = None
        self.monitor_imu = False

    def triangle_buoy_pose_callback(self, msg):
        # Set the first pose and don't abort
        if self.triangle_buoy_pose == None:
            self.triangle_buoy_pose = msg
            return
        change_in_pose = self.get_distance(self.triangle_buoy_pose.pose.position, msg.pose.position)
        if (change_in_pose > self.replan_threshold) and self.started:
            rospy.logwarn_throttle(1,"Triangle Buoy Pose jump.")
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

    @staticmethod
    def get_approach_pose(cur_pose, buoy_pose, orbit_radius):
        """
        Calculate approach pose to begin circling the buoy

        Parameters
        ----------
        self.triangle_buoy_pose : PoseStamped
            Triangle buoy's pose

        Returns
        -------
        approach_pose : Pose
        """
        # Find line from sub to buoy
        if ( round(cur_pose.position.x, 2) == round(buoy_pose.position.x,2) ): # Avoid infinite slope in the polyfit
            buoy_pose.position.x += 0.1
        if ( round(cur_pose.position.y, 2) == round(buoy_pose.position.y,2) ): # Avoid infinite slope in the polyfit
            buoy_pose.position.y -= 0.1

        x_new = buoy_pose.position.x
        y_new = buoy_pose.position.y

        # Adjust buoy pose behind the buoy
        x2 = np.array([cur_pose.position.x, x_new])
        y2 = np.array([cur_pose.position.y, y_new])
        m2, b2 = np.polyfit(x2,y2,1)
        m2 = round(m2, 2)
        b2 = round(b2, 2)
        x_hat2 = np.sqrt( ( orbit_radius**2) / (m2**2 + 1) )
        if x_new > cur_pose.position.x:
            x_hat2 = -x_hat2
        x_new2 = x_new + x_hat2
        y_new2 = x_new2 * m2 + b2

        target_pose = Pose()
        target_pose.position.x = x_new2
        target_pose.position.y = y_new2
        target_pose.position.z = buoy_pose.position.z
        return target_pose

    def execute(self, userdata):
        """
        Approach bouy
        Make orbit actionlib req, success case is class detected
        Task code cancels the req
        proceed to slaying and backing up
        """
        self.started = True
        self.clear_abort()
        self.configure_darknet_cameras([1,0,0,0,0,0])
        approach_pose = self.get_approach_pose(self.cur_pose, self.triangle_buoy_pose.pose, self.approach_dist)
        if self.go_to_pose(approach_pose):
            return "aborted"        
        return "success"