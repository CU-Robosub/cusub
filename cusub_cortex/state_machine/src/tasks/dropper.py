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

class Dropper(Task):
    name = "dropper"

    def __init__(self):
        self.name = "dropper"
        super(Dropper, self).__init__()
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        darknet_cameras = [1,0,0,0,0,1]
        search_frames = ["leviathan/description/occam0_frame_optical", "leviathan/description/downcam_frame_optical"]
        search_classes = ["wolf", "dropper_cover"]
        # search_frames = ["leviathan/description/downcam_frame_optical"]
        self.search = Search.from_bounding_box(self.get_prior_param(), search_classes, search_frames, darknet_cameras)
        self.drop = Drop()

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Drop', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
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
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)
        rospy.loginfo("...waiting for visual_servo server")
        self.vs_client.wait_for_server()
        rospy.loginfo("\tfound visual servo server")
        self.darknet_classes = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
        
        # variables
        self.centering_time = rospy.get_param("tasks/dropper/centering_time", 1.0)
        self.feedback = False
        self.was_centered = False

        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        self.depth_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depth_callback)
        
        self.drive_pub = rospy.Publisher("cusub_common/motor_controllers/pid/drive/setpoint", Float64, queue_size=1)
        self.drive_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/drive/state", Float64, self.drive_callback)
        self.last_drive = 0
        
        self.drive_carrot = rospy.get_param("tasks/dropper/drive_carrot")
        self.drop_depth = rospy.get_param("tasks/dropper/drop_depth")
        self.approach_depth = rospy.get_param("tasks/dropper/approach_depth")
        self.drop_depth_thresh = rospy.get_param("tasks/dropper/drop_depth_thresh")
        self.target_pixel_threshold = rospy.get_param("tasks/dropper/target_pixel_threshold")
        self.drop_timeout = rospy.get_param("tasks/dropper/drop_timeout")
        self.depth_carrot = rospy.get_param("tasks/dropper/depth_carrot")

        super(Drop, self).__init__(self.outcomes, "drop")

    # dropper 0 = left
    # dropper 1 = dright
    def actuate_dropper(self, dropper_num):
        self.actuator_service(dropper_num, 500)

    def vs_feedback_callback(self, feedback):
        self.feedback = feedback.centered
    
    def depth_callback(self, depth):
        self.last_depth = depth.data

    def drive_callback(self, drive):
        self.last_drive = drive.data

    def visual_servo_method(self, userdata):
        rospy.loginfo("...using visual servoing approach")
        
        self.configure_darknet_cameras([1,0,0,0,0,1])
        target_classes = ["wolf", "bat", "dropper_cover"]
        
        goal = VisualServoGoal()
        goal.target_classes = target_classes
        goal.camera = goal.OCCAM
        goal.x_axis = goal.YAW_AXIS
        goal.y_axis = goal.DRIVE_AXIS
        goal.area_axis = goal.NO_AXIS
        goal.target_frame = rospy.get_param("~robotname") +"/description/occam0_frame_optical"
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_pixel_x = goal.CAMERAS_CENTER_X
        goal.target_pixel_y = goal.CAMERAS_CENTER_Y * 2
        goal.target_box_area = 0.3
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
        rospy.loginfo("...moved to the dropper detections, about to center using downcam")

        # toggle to using the downcam to center
        goal = VisualServoGoal()
        goal.visual_servo_type = goal.PROPORTIONAL
        goal.target_classes = ["wolf", "bat"]
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

        # Drop down to low above the ground (get it low)
        # NEED THIS TO BE GRADUAL!
        # depth_set.data = self.drop_depth
        
        self.vs_client.send_goal(goal, feedback_cb=self.vs_feedback_callback)
        
        userdata.timeout_obj.set_new_time(self.drop_timeout)

        count = 0
        while not rospy.is_shutdown():
            if userdata.timeout_obj.timed_out:
                print("Timeout!!")
                break # timeout, drop anyway, cause why not
            # only begin lowering when centered in downcam
            if self.feedback:
                # sim:
                # count += 1

                # if count > 20:
                #     break
                
                #self.depth_pub.publish(depth_set)

                new_depth = max(self.last_depth - self.depth_carrot, self.drop_depth)
                self.depth_pub.publish(new_depth)

                if abs(self.last_depth - self.drop_depth) < self.drop_depth_thresh:
                    break

            rospy.sleep(0.25)
        
        rospy.loginfo("About to drop!")
        self.actuate_dropper(0)
        rospy.sleep(5)
        rospy.loginfo("About to drop again!")
        self.actuate_dropper(1)

        self.vs_client.cancel_goal()
        userdata.outcome = "success"
        return "success"

    def execute(self, userdata):
        return self.visual_servo_method(userdata)
