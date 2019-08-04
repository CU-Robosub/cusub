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
from darknet_multiplexer.srv import DarknetClasses

TRIANGLE_BOUY_CLASSES = ["vampire_fathead", "vampire_flying", "vampire_no_torso"]

class Triangle(Task):
    name = "triangle"

    def __init__(self):
        super(Triangle, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        frame = ["leviathan/description/occam0_frame_optical"]
        self.search = Search.from_bounding_box(self.get_prior_param(), TRIANGLE_BOUY_CLASSES, frame)
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
        self.do_orbit = rospy.get_param("tasks/triangle/do_orbit")
        self.target_vampire = rospy.get_param("tasks/triangle/target_vampire")

        rospy.loginfo("...waiting for darknet get classes server")
        rospy.wait_for_service("cusub_perception/darknet_multiplexer/get_classes")
        self.get_classes_service = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
        rospy.loginfo("\tfound darkner get classes server")

        # strafe info
        self.orbit_right = rospy.get_param("tasks/triangle/orbit_right")
        self.strafe_carrot = rospy.get_param("tasks/triangle/strafe_carrot")
        self.strafe_state = None
        rospy.Subscriber("cusub_common/motor_controllers/pid/strafe/setpoint", Float64, self.strafe_callback)
        self.strafe_pub = rospy.Publisher("cusub_common/motor_controllers/pid/strafe/setpoint", Float64, queue_size=10)

        self.single_class_time = rospy.get_param("tasks/triangle/single_class_time")

        rospy.loginfo("...waiting for toggle waypoint control")
        rospy.wait_for_service('cusub_common/toggleWaypointControl')
        self.wayToggle = rospy.ServiceProxy('cusub_common/toggleWaypointControl', ToggleControl)
        rospy.loginfo("\tfound toggle waypoint control")

        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=10)

    def drive_callback(self, msg):
        self.drive_state = msg.data
    
    def strafe_callback(self, msg):
        self.strafe_state = msg.data

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
            # if rospy.get_param("using_sim_params"): # go to a constant depth
            #     depth_msg = Float64()
            #     depth_msg.data = -2.0
            #     self.depth_pub.publish(depth_msg)
                
            rospy.sleep(0.25)
        
        rospy.loginfo("\tcentered")

        # Orbit
        userdata.timeout_obj.set_new_time(30)
        if self.do_orbit:
            target_frame = ["leviathan/description/occam0_frame_optical"]
            strafe_msg = Float64()
            found = False
            while not rospy.is_shutdown():
                # Adjust strafe with carrot on current strafe
                strafe_set = 0.0 if self.strafe_state is None else self.strafe_state
                if self.orbit_right:
                    strafe_msg.data = strafe_set + self.strafe_carrot
                else:
                    strafe_msg.data = strafe_set - self.strafe_carrot
                
                # Check available darknet classes in occam0 frame, hang for 1 second
                present_classes = self.get_classes_service(rospy.Duration(0.5), target_frame).classes
                if len(present_classes) == 1 and self.target_vampire in present_classes:
                    rospy.loginfo("Target class detectde by itself. Waiting...")
                    if not found:
                        userdata.timeout_obj.set_new_time(self.single_class_time)
                        found = True
                
                if userdata.timeout_obj.timed_out:
                    break

                self.strafe_pub.publish(strafe_msg)

        rospy.sleep(3) # Provide time to fully center on the bouy
        self.vs_client.cancel_goal()
    
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
        userdata.timeout_obj.set_new_time(2* rospy.get_param("tasks/jiangshi/visual_servo/slay_timeout"))
        # slay_set.data = original_set
        while not rospy.is_shutdown():
            slay_set.data = self.drive_state - slay_carrot
            self.drive_pub.publish(slay_set)
            if userdata.timeout_obj.timed_out:
                break
            rospy.sleep(0.25)
        self.wayToggle(True)

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
        else:
            return self.visual_servo_method(userdata)
        
