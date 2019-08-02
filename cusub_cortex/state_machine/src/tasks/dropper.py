#!/usr/bin/env python
from __future__ import division
"""
Dropper task, attempts to center on the dropper, set the depth lower, and drop
Objectives:
- Search
- Follow
---> Center on the dropper
---> Lower the depth
---> Drop
"""
from tasks.task import Task, Objective
from tasks.search import Search
import numpy as np
import rospy
import smach
import smach_ros
from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback
import actionlib
from actuator.srv import ActivateActuator
from std_msgs.msg import Float64
from state_machine.srv import *
import tf

TARGET_CLASS = "coffin"

class Dropper(Task):
    name = "dropper"

    def __init__(self):
        self.name = "dropper"
        super(Dropper, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search.from_bounding_box(self.get_prior_param(), TARGET_CLASS, [0,0,0,0,0,1])
        self.drop = Drop()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Drop', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            # smach.StateMachine.add('Get_Low', self.follow, transitions={'success':'Drop', 'timed_out':'manager'}, \
            #     remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Drop', self.drop, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Drop(Objective):
    """
    Center, get low, drop 
    """
    outcomes=['success', 'timed_out']

    def __init__(self):
        self.vs_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")
        self.centering_time = rospy.get_param("tasks/dropper/centering_time", 1.0)
        self.feedback = False
        self.was_centered = False
        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        self.depth_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depth_callback)
        self.last_depth = Float64()
        self.depth_msg = Float64()
        self.depth_msg.data = rospy.get_param("tasks/dropper/search_depth")
        self.drop_depth = rospy.get_param("tasks/dropper/drop_depth")
        self.drop_thresh = rospy.get_param("tasks/dropper/drop_thresh")
        self.target_pixel_box = rospy.get_param("tasks/dropper/target_pixel_threshold")
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)

        super(Drop, self).__init__(self.outcomes, "drop")

    def actuate_dropper(self, _):
        self.actuater_service(1, 500)

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered
    
    def depth_callback(self, depth):
        self.last_depth.data = depth.data

    def visual_servo_method(self, userdata):
   #     self.configure_darknet_cameras([1,0,0,0,0,0])
        rospy.loginfo("...using visual servoing approach")
        goal = VisualServoGoal()
        goal.approach_target_class = ["wolf", "bat", "dropper_cover", "lever"]
        goal.target_class = ["wolf", "bat"]
        goal.camera = goal.OCCAM
        goal.x_axis = goal.YAW_AXIS
        goal.y_axis = goal.DEPTH_AXIS
        goal.area_axis = goal.DRIVE_AXIS
        goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = 0
        goal.target_box_area = goal.FORTY_PERCENT_IMAGE
        goal.target_pixel_threshold = self.target_pixel_box
        rospy.loginfo("...moving to box")
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)

        while not rospy.is_shutdown() :
            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            rospy.sleep(0.25)
        self.vs_client.cancel_goal()
        rospy.loginfo("\tmoved to the dropper detections")

        # wait for more downcam feedback?

        # toggle to using the downcam to center
        goal.camera = goal.DOWNCAM
        goal.target_box_area = goal.TWENTY_PERCENT_IMAGE
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        # todo MM: double check the downcam frame name
        # She'll be 'right
        goal.target_frame = rospy.get_param("~robotname") +"/description/downcam_frame_optical"
        goal.x_axis = goal.STRAFE_AXIS
        goal.y_axis = goal.DRIVE_AXIS
        goal.area_axis = goal.NO_AXIS
        goal.target_box_area = goal.AREA_NOT_USED
        

        # Drop down to low above the ground (get it low)
        self.wayToggle(False)
        depth_set = Float64()
        drive_set.data = self.drop_depth
        userdata.timeout_obj.set_new_time(rospy.get_param("tasks/dropper/drop_timeout"))
        while not rospy.is_shutdown():
            self.depth_pub.publish(depth_set)
            if userdata.timeout_obj.timed_out:
                break # on a timeout, drop anyway, cause why not
            rospy.sleep(0.25)

        rospy.loginfo("About to drop!")
        self.actuate_dropper()

        self.wayToggle(True)
        userdata.outcome = "success"
        return "success"

    def execute(self, userdata):
        return self.visual_servo_method()
