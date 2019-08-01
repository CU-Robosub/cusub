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

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered
    
    def depth_callback(self, depth):
        self.last_depth.data = depth.data

    def execute(self, userdata):
        goal = VisualServoGoal()
        goal.target_class = TARGET_CLASS # todo SK
        goal.camera = goal.DOWNCAM
        goal.target_frame = rospy.get_param("~robotname") +"/camera"
        goal.target_frame = rospy.get_param("~robotname") +"/description/downcam_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_pixel_threshold = self.target_pixel_box
        rospy.loginfo("...centering over path marker")
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)
        
        while not rospy.is_shutdown() :
            self.depth_pub.publish(self.depth_msg)
            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            if self.feedback:
                if self.was_centered:
                    break
                else:
                    rospy.sleep(self.centering_time)
                    self.was_centered = True
            else:
                self.was_centered = False
            rospy.sleep(0.25)
        rospy.loginfo("\tcentered")
    
        # keep pubbing until we get there
        self.depth_msg.data = self.drop_depth
        while abs(self.last_depth.data - self.depth_msg.data) > self.drop_thresh:
            self.depth_pub.publish(self.depth_msg)
            rospy.sleep(0.25)

        rospy.loginfo("...centering over path marker")
        while not rospy.is_shutdown() :
            self.depth_pub.publish(self.depth_msg)
            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            if self.feedback:
                if self.was_centered:
                    break
                else:
                    rospy.sleep(5)
                    self.was_centered = True
            else:
                self.was_centered = False
            rospy.sleep(0.25)
        rospy.loginfo("\tcentered (again)")
        rospy.loginfo("...about to drop my load ;)")
        self.actuator_service(1, 500)
        rospy.loginfo("\tdropped")


        self.vs_client.cancel_goal()  # Tell VS to stop servoing
        userdata.outcome = "success"
        return "success"
