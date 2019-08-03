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
import numpy as np

import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import Float64
import tf

from tasks.search import Search
from tasks.task import Task, Objective

from perception_control.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback

from actuator.srv import ActivateActuator
from darknet_multiplexer.srv import DarknetClasses

class Debug(Task):
    name = "debug_servo"

    def __init__(self):
        self.name = "debug_servo"
        super(Dropper, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.drop = Drop()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Drop', self.drop, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Drop(Objective):
    """
    Center, get low, drop 
    """
    outcomes=['success', 'timed_out']

    def __init__(self):
        # interfaces
        self.vs_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")

        rospy.loginfo("...waiting for actuator service")
        rospy.wait_for_service("cusub_common/activateActuator")
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)
        rospy.loginfo("\tfound actuator service")
        
        # variables
        self.centering_time = rospy.get_param("tasks/dropper/centering_time", 1.0)
        self.dive_feedback = False

        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        self.target_pixel_threshold = rospy.get_param("tasks/dropper/target_pixel_threshold")
        self.drop_depth = rospy.get_param("tasks/dropper/drop_depth")

        super(Drop, self).__init__(self.outcomes, "Drop")

    def actuate_dropper(self, dropper_num):
        self.actuator_service(dropper_num, 500)

    def visual_servo_method(self, userdata):
        rospy.loginfo("...using visual servoing approach")
        
        # toggle to using the downcam to 
        self.configure_darknet_cameras([0,0,0,0,0,1])
        goal = VisualServoGoal()
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_classes = ["vampire_cute"]
        goal.camera = goal.DOWNCAM
        goal.target_pixel_threshold = self.target_pixel_threshold
        goal.target_frame = rospy.get_param("~robotname") +"/description/downcam_frame_optical"
        goal.x_axis = goal.STRAFE_AXIS
        goal.y_axis = goal.DRIVE_AXIS
        goal.area_axis = goal.NO_AXIS
        if rospy.get_param("using_sim_params"):
            goal.target_pixel_x = goal.DOWNCAM_FAKE_CENTER_X
            goal.target_pixel_y = goal.DOWNCAM_FAKE_CENTER_Y
        else: # real sub
            goal.target_pixel_x = goal.CAMERAS_CENTER_X
            goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_box_area = goal.AREA_NOT_USED
        self.vs_client.send_goal(goal, feedback_cb=self.vs_dive_feedback_callback)
        rospy.loginfo("...centering with downcam")
        
        depth_set = Float64()
        depth_set.data = self.last_depth
        while not rospy.is_shutdown():
            # begin rising if centered in downcam
            if self.dive_feedback:
                rospy.sleep(2)
                rospy.loginfo_throttle(1, "...centered, adjusting depth")
                depth_set.data = self.last_depth - self.depth_carrot
            
            if self.last_depth < self.drop_depth:
                depth_set.data = self.last_depth
                self.depth_pub.publish(depth_set)
                rospy.loginfo("...dropping")
                break
                
            self.depth_pub.publish(depth_set)
            rospy.sleep(0.25) # don't eat the core
        
        self.actuate_dropper(1)
        rospy.sleep(5) # center and wait for dropper
        if not rospy.get_param("using_sim_params"):
            self.actuate_dropper(0)

        self.vs_client.cancel_goal()
        rospy.sleep(3)
        userdata.outcome = "success"
        return "success"
   
    def execute(self, userdata):
        return self.visual_servo_method(userdata)
