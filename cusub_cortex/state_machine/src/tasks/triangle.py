#!/usr/bin/env python
from __future__ import division
"""
Triangle Buoy Task
"""
from tasks.task import Task, Objective
from tasks.search import Search
import tf
import numpy as np
import rospy
import smach
import smach_ros
import copy
from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback
import actionlib
from std_msgs.msg import Float64
from waypoint_navigator.srv import *
from geometry_msgs.msg import Pose

TRIANGLE_BOUY_CLASSES = ["vampire_fathead", "vampire_flying", "vampire_no_torso"]

class Triangle(Task):
    name = "triangle"

    def __init__(self):
        super(Triangle, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        frame = ["leviathan/description/occam0_frame_optical"]
        self.search = Search.from_bounding_box(self.get_prior_param(), TRIANGLE_BOUY_CLASSES, frame, [1,1,1,1,1,0])
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
    outcomes=['success', 'timed_out']

    def __init__(self):
        super(Slay, self).__init__(self.outcomes, "Slay")

        self.feedback = None
        self.target_pixel_box = rospy.get_param("tasks/triangle/visual_servo/target_pixel_threshold")
        self.vs_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")

        self.drive_state = None
        rospy.Subscriber("cusub_common/motor_controllers/pid/drive/state", Float64, self.drive_callback)
        self.drive_pub = rospy.Publisher("cusub_common/motor_controllers/pid/drive/setpoint", Float64, queue_size=1)

        rospy.loginfo("...waiting for toggle waypoint control")
        rospy.wait_for_service('cusub_common/toggleWaypointControl')
        self.wayToggle = rospy.ServiceProxy('cusub_common/toggleWaypointControl', ToggleControl)
        rospy.loginfo("\tfound toggle waypoint control")

    def drive_callback(self, msg):
        self.drive_state = msg.data

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered

    def visual_servo_method(self, userdata):
        self.configure_darknet_cameras([1,0,0,0,0,0])
        rospy.loginfo("...using visual servoing approach")
        goal = VisualServoGoal()
        goal.target_classes = TRIANGLE_BOUY_CLASSES
        goal.camera = goal.OCCAM
        goal.x_axis = goal.YAW_AXIS
        goal.y_axis = goal.DEPTH_AXIS
        goal.area_axis = goal.DRIVE_AXIS
        goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_box_area = (int) ( goal.ONE_HUNDRED_PERCENT_IMAGE * rospy.get_param("tasks/jiangshi/visual_servo/goal_percentage_of_frame") )
        goal.target_pixel_threshold = self.target_pixel_box
        rospy.loginfo("...centering")
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)

        while not rospy.is_shutdown() :
            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            if self.feedback:
                break
            rospy.sleep(0.25)
        self.vs_client.cancel_goal()
        rospy.loginfo("\tcentered")

        # Slay the buoy
        rospy.sleep(2) # Wait for the vs client to release control to the waypoint nav for us to take it back again
        rospy.loginfo("...slaying")
        try:
            res = self.wayToggle(False)
            rospy.loginfo(res)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s", e)
        slay_set = Float64()
        original_set = copy.deepcopy(self.drive_state)
        userdata.timeout_obj.set_new_time(rospy.get_param("tasks/jiangshi/visual_servo/slay_timeout"))
        slay_carrot = rospy.get_param("tasks/jiangshi/visual_servo/slay_carrot")
        while not rospy.is_shutdown():
            slay_set.data = self.drive_state + slay_carrot
            self.drive_pub.publish(slay_set)
            if userdata.timeout_obj.timed_out:
                break
            rospy.sleep(0.25)

        rospy.loginfo("...slayed, backing up")
        userdata.timeout_obj.set_new_time(3* rospy.get_param("tasks/jiangshi/visual_servo/slay_timeout"))
        # slay_set.data = original_set
        while not rospy.is_shutdown():
            slay_set.data = self.drive_state - slay_carrot
            self.drive_pub.publish(slay_set)
            if userdata.timeout_obj.timed_out:
                break
            rospy.sleep(0.25)
        self.wayToggle(True)

        # go to pose convenient for droppers
        # userdata.timeout_obj.timed_out.set_new_time(30)
        # evasion_pose = Pose()
        # evasion_pose.position.x = -21.8
        # evasion_pose.position.y = 6.6
        # evasion_pose.position.z = -1.8
        # if self.go_to_pose(evasion_pose, userdata.timeout_obj, replan_enabled=False):
        #     if userdata.timeout_obj.timed_out:
        #         userdata.outcome = "timed_out"
        #         return "timed_out"

        userdata.outcome = "success"
        return "success"

    def mikes_method(self, userdata):
        rise_pose = copy.deepcopy(self.cur_pose)
        prev_z = rise_pose.position.z
        rise_pose.position.z = rospy.get_param("tasks/triangle/mikes_method/go_over_jiangshi_depth")
        if self.go_to_pose(rise_pose, userdata.timeout_obj, replan_enabled=False):
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "timed_out"
        quat = [self.cur_pose.orientation.x, self.cur_pose.orientation.y,self.cur_pose.orientation.z,self.cur_pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        meters_fwd = rospy.get_param("tasks/triangle/mikes_method/meters_fwd")
        fly_pose = Pose()
        fly_pose.orientation = self.cur_pose.orientation
        fly_pose.position.x = self.cur_pose.position.x - meters_fwd * np.cos(yaw)
        fly_pose.position.y = self.cur_pose.position.y - meters_fwd * np.sin(yaw)
        fly_pose.position.z = self.cur_pose.position.z
        if self.go_to_pose(fly_pose, userdata.timeout_obj, replan_enabled=False):
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "timed_out"
        drop_pose = copy.deepcopy(self.cur_pose)
        drop_pose.position.z = prev_z
        quat = tf.transformations.quaternion_from_euler(0,0, yaw + np.pi)
        drop_pose.orientation.x = quat[0]
        drop_pose.orientation.x = quat[1]
        drop_pose.orientation.x = quat[2]
        drop_pose.orientation.x = quat[3]
        if self.go_to_pose(drop_pose, userdata.timeout_obj, replan_enabled=False):
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "timed_out"
        self.visual_servo_method(userdata)

    def execute(self, userdata):
        if rospy.get_param("tasks/triangle/mikes_method/do"):
            self.mikes_method(userdata)
        elif rospy.get_param("tasks/triangle/do_orbit"):
            pass
        else:
            return self.visual_servo_method(userdata)
        
