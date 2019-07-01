#!/usr/bin/env python
"""
Triangule Buoy Task, attempts to slay a particular class on the triangle buoy
Objectives:
- Search
- Slay
---> Go to approach point
---> Slay vampire
---> Backup
"""
from tasks.task import Task, Objective
import numpy as np

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

    def get_approach_pose(self):
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
        approach_pose = Pose()
        quat = [self.triangle_buoy_pose.pose.orientation.x, self.triangle_buoy_pose.pose.orientation.y,self.triangle_buoy_pose.pose.orientation.z,self.triangle_buoy_pose.pose.orientation.w]
        triangle_buoy_roll, triangle_buoy_pitch, triangle_buoy_yaw = tf.transformations.euler_from_quaternion(quat)
        approach_pose.position.x = self.triangle_buoy_pose.pose.position.x + self.approach_dist * np.cos(triangle_buoy_yaw)
        approach_pose.position.y = self.triangle_buoy_pose.pose.position.y + self.approach_dist * np.sin(triangle_buoy_yaw)
        approach_pose.position.z = self.triangle_buoy_pose.pose.position.z
        goal_quat = tf.transformations.quaternion_from_euler(triangle_buoy_roll, triangle_buoy_pitch, triangle_buoy_yaw)
        approach_pose.orientation.x = goal_quat[0]
        approach_pose.orientation.x = goal_quat[1]
        approach_pose.orientation.x = goal_quat[2]
        approach_pose.orientation.x = goal_quat[3]
        return approach_pose

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
        approach_pose = self.get_approach_pose()
        if self.go_to_pose(approach_pose):
            return "aborted"        
        return "success"