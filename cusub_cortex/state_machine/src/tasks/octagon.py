#!/usr/bin/env python
from __future__ import division
"""
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

class Octagon(Task):
    name = "octagon"

    def __init__(self):
        self.name = "octagon"
        super(Octagon, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        darknet_cameras = [1,0,0,0,0,1]
        search_frames = ["leviathan/description/occam0_frame_optical", "leviathan/description/downcam_frame_optical"]
        search_classes = ["coffin"]
        self.search = Search.from_bounding_box(self.get_prior_param(), search_classes, search_frames, darknet_cameras)
        self.octagon = Octagon()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Octagon', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Octagon', self.octagon, transitions={'success':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Octagon(Objective):
    """
    Center, get low, drop 
    """
    outcomes=['success', 'timed_out']

    def __init__(self):
        # interfaces
        self.vs_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")
        self.darknet_classes = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
        
        # variables
        self.centering_time = rospy.get_param("tasks/octagon/centering_time", 1.0)
        self.feedback = False
        self.was_centered = False

        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        self.depth_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depth_callback)
        
        self.drive_pub = rospy.Publisher("cusub_common/motor_controllers/pid/drive/setpoint", Float64, queue_size=1)
        self.drive_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/drive/state", Float64, self.drive_callback)
        self.last_drive = 0
        
        self.approach_depth = rospy.get_param("tasks/octagon/approach_depth")
        self.target_pixel_threshold = rospy.get_param("tasks/octagon/target_pixel_threshold")
        self.drop_timeout = rospy.get_param("tasks/octagon/drop_timeout")
        self.depth_carrot = rospy.get_param("tasks/octagon/depth_carrot")
        self.rise_depth = rospy.get_param("tasks/octagon/rise_depth")

        super(Octagon, self).__init__(self.outcomes, "drop")

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered
    
    def depth_callback(self, depth):
        self.last_depth = depth.data

    def drive_callback(self, drive):
        self.last_drive = drive.data

    def visual_servo_method(self, userdata):
        rospy.loginfo("...using visual servoing approach")
        
        self.configure_darknet_cameras([1,0,0,0,0,1])
        target_classes = ["coffin"]
        
        goal = VisualServoGoal()
        goal.target_classes = target_classes
        goal.camera = goal.OCCAM
        goal.x_axis = goal.YAW_AXIS
        goal.y_axis = goal.DRIVE_AXIS
        goal.area_axis = goal.NO_AXIS
        goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.NO_AXIS
        goal.target_box_area = goal.AREA_NOT_USED
        goal.target_pixel_threshold = self.target_pixel_threshold
        
        depth_set = Float64()
        depth_set.data = self.approach_depth

        drive_set = Float64()
        drive_set.data = self.last_drive
        
        rospy.loginfo("...moving to box")
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)

        while not rospy.is_shutdown() :
             # check if downcam can see it

            found = False
            for _class in self.darknet_classes(rospy.Duration(1), ["leviathan/description/downcam_frame_optical"]).classes:
                if _class in target_classes:
                    found = True
            
            if found: # getting downcam hits
                break


            if userdata.timeout_obj.timed_out:
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"

            # pub out constant drive`
            drive_set.data = self.last_drive + self.drive_carrot
            self.drive_pub.publish(drive_set)

            self.depth_pub.publish(depth_set)
            
            rospy.sleep(0.25)

        self.vs_client.cancel_goal()
        rospy.loginfo("...found coffin in downcam, about to center")

        # toggle to using the downcam to center
        self.configure_darknet_cameras([0,0,0,0,0,1])
        goal = VisualServoGoal()
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_classes = ["coffin"]
        goal.camera = goal.DOWNCAM

        # in sim:
        # goal.target_pixel_x = goal.DOWNCAM_FAKE_CENTER_X
        # goal.target_pixel_y = goal.DOWNCAM_FAKE_CENTER_Y

        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y
        goal.target_pixel_threshold = self.target_pixel_threshold
        goal.target_frame = rospy.get_param("~robotname") +"/description/downcam_frame_optical"
        goal.target_box_area = goal.AREA_NOT_USED
        goal.x_axis = goal.STRAFE_AXIS
        goal.y_axis = goal.DRIVE_AXIS
        goal.area_axis = goal.NO_AXIS

        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)
        
        userdata.timeout_obj.set_new_time(self.drop_timeout)

        while not rospy.is_shutdown():
            if userdata.timeout_obj.timed_out:
                print("Timeout!!")
                self.vs_client.cancel_goal()
                userdata.outcome = "timed_out"
                return "timed_out"
            # only begin rising if centered in downcam
            if self.feedback:
                print("... Rising up...")
                new_depth = min(self.last_depth + self.depth_carrot, self.rise_depth)
                self.depth_pub.publish(new_depth)

                if abs(self.last_depth - self.rise_depth) < self.drop_depth_thresh:
                    print("... Done rising!")
                    self.vs_client.cancel_goal()
                    userdata.outcome = "success"
                    return "success"

            rospy.sleep(0.25)
        
        rospy.loginfo("Should not be here")
   
    def execute(self, userdata):
        return self.visual_servo_method(userdata)
