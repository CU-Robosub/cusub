#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function
"""
Torpedos Task, attempts to pump some lead into Torpedos
Objectives:
- Search
- Approach
- Slay
- Back Up
"""
from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import tf
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Time
from detection_listener.listener import DetectionListener
from waypoint_navigator.srv import ToggleControl
import numpy as np
from cusub_print.cuprint import bcolors
import sys

from actuator.srv import ActivateActuator

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv

from cv_bridge import CvBridge

MIN_MATCH_COUNT = 10

class Torpedos(Task):
    name = "Torpedos"

    def __init__(self):
        super(Torpedos, self).__init__(self.name)

        # All Objectives share the same listener to gaurantee same data between objectives
        self.listener = DetectionListener() 

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        search_classes = ["dracula"]
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param())
        self.approach = Approach(self.name, self.listener)
        self.slay = Slay(self.name)
        self.retrace = Retrace(self.name, self.listener)
        self.backup = Backup(self.name)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Slay', self.slay, transitions={'slayed':'Backup', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Backup', self.backup, transitions={'backed_up':'manager', 'timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'Slay', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']

    target_class_id = "dracula"

    def __init__(self, task_name, listener):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = PIDClient(name, "drive")
        self.depth_client = PIDClient(name, "depth")

        self.retrace_timeout = rospy.get_param("tasks/" + task_name.lower() + "/retrace_timeout", 2)
        seconds = rospy.get_param("tasks/" + task_name.lower() + "/seconds_in_position", 2)
        self.rate = rospy.get_param("tasks/" + task_name.lower() + "/new_dv_check_rate", 30)
        self.count_target = seconds * self.rate
        self.count = 0
        
        self.approach_stop_dist = rospy.get_param("tasks/" + task_name.lower() + "/approach_stop_dist")
        self.approach_carrot_dist = rospy.get_param("tasks/" + task_name.lower() + "/approach_carrot_dist")
        self.height2dist_slope = rospy.get_param("tasks/" + task_name.lower() + "/height2dist_slope")
        self.height2dist_intercept = rospy.get_param("tasks/" + task_name.lower() + "/height2dist_intercept")
    
    def execute(self, userdata):
        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        # Find vampire_cute's dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target_class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.cuprint("selecting the first", warn=True)
        elif not dobj_nums: # Chck if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        dobj_num = dobj_nums[0]
        self.cuprint("located " + self.target_class_id + "'s dobject num: " + bcolors.HEADER + str(dobj_num) + bcolors.ENDC)
        
        # Start a watch dog timer on detections
        watchdog_timer = Timeout(self.name + "/Retrace Watchdog")

        # Enable the PID loops
        self.yaw_client.enable()
        self.drive_client.enable()
        self.depth_client.enable()

        self.cuprint("servoing")
        r = rospy.Rate(self.rate)
        print("")
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)
                [az, el, height, width] = self.listener.get_avg_bearing(dobj_num, num_dv=3)
                if az == None: # if secs is set in get_avg_bearing we could have a new dv it just wasn't in recent time frame
                    r.sleep()
                    continue
                self.yaw_client.set_setpoint(az, loop=False)

                # Calculate translational errors
                dist = self.get_Torpedos_distance(height)
                [delta_x, delta_y, delta_z] = self.get_relative_Torpedos_xyz(az, el, dist)
                xy_dist = np.linalg.norm([delta_x, delta_y])

                # set depth setpoint
                self.depth_client.set_setpoint(self.depth_client.get_standard_state() + delta_z, loop=False)
                
                # pretty printing
                error_string_dist = "Error xy: " + bcolors.HEADER + str(round(xy_dist,2)) +bcolors.ENDC
                error_string_z = " | z: " + bcolors.HEADER + str(round(delta_z,2)) + bcolors.ENDC
                self.cuprint(error_string_dist + error_string_z, print_prev_line=True)

                # Set drive setpoint and check if in position
                if xy_dist > self.approach_stop_dist:
                    self.drive_client.set_setpoint(self.drive_client.get_standard_state() + self.approach_carrot_dist, loop=False)
                else:
                    self.drive_client.set_setpoint(self.drive_client.get_standard_state())
                    break

            elif watchdog_timer.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                return "lost_object"

            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"
            r.sleep()

        watchdog_timer.timer.shutdown()
        self.yaw_client.disable()
        self.drive_client.disable()
        self.depth_client.disable()
        return "in_position"

    def get_relative_Torpedos_xyz(self, az, el, dist):
        # Converts spherical coordinates to x,y,z
        z = dist * np.sin(-el) # flip elevation sign
        xy_dist = dist * np.cos(el)
        x = xy_dist * np.cos(az)
        y = xy_dist * np.sin(az)
        return [x,y,z]
    
    def get_Torpedos_distance(self, Torpedos_height):
        # Solve linear system
        return self.height2dist_slope * Torpedos_height + self.height2dist_intercept

class Slay(Objective):
    outcomes = ['slayed', 'timed_out']

    def __init__(self, task_name):

        name = task_name + "/Slay"

        super(Slay, self).__init__(self.outcomes, name)

        self.drive_client = PIDClient(name, "drive")
        self.depth_client = PIDClient(name, "depth")
        self.strafe_client = PIDClient(name, "strafe")

        self.slay_carrot = rospy.get_param("tasks/" + task_name.lower() + "/slay_carrot")
        self.approach_stop_dist = rospy.get_param("tasks/" + task_name.lower() + "/approach_stop_dist")

        self.rate = 10

        self.bridge = CvBridge()

        # Initiate SIFT detector
        self.sift = cv.xfeatures2d.SIFT_create()

        img1_fn = '/opt/dracula.png'
        self.img1 = cv.imread(img1_fn,0) # queryImage
        self.kp1, self.des1 = self.sift.detectAndCompute(self.img1,None)

        self.occam_camera_matrix = np.array([656.926137, 0.000000, 376.5, 0.000000, 656.926137, 240.5, 0.000000, 0.000000, 1.000000])
        self.occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000])
        self.occam_camera_matrix.shape = (3,3)

        self.debug_pub = rospy.Publisher('homography_debug', Image, queue_size=1)

        rospy.Subscriber('/leviathan/cusub_common/occam/image0', Image, self.match_image_callback, queue_size=1, buff_size=2**24)

        self.cuprint("waiting for actuator service")
        rospy.wait_for_service("cusub_common/activateActuator")
        self.cuprint("...connected")
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)

        self.offset = None

    def match_image_callback(self, image):

        img2 = self.bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')

        # find the keypoints and descriptors with SIFT
        kp2, des2 = self.sift.detectAndCompute(img2,None)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:

            src_pts = np.float32([ self.kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            model_points = []
            image_points = []
            for match in good:
                #print(self.kp1[match.imgIdx].pt, kp2[match.queryIdx].pt)
                # Adjust model points to center at target
                model_points.append([(self.kp1[match.queryIdx].pt[0] - 260.0) / 556.0, (self.kp1[match.queryIdx].pt[1] - 80.0) / 556.0, 0.0])
                image_points.append([float(kp2[match.trainIdx].pt[0]), float(kp2[match.trainIdx].pt[1])])

            model_points = np.array(model_points)
            image_points = np.array(image_points).reshape((len(image_points),1,2))

            (success, rotation_vector, translation_vector, _) = cv.solvePnPRansac(
            model_points, image_points, self.occam_camera_matrix, self.occam_distortion_coefs,
            reprojectionError=5.0, flags=cv.SOLVEPNP_ITERATIVE)

            self.offset = (translation_vector[0], translation_vector[1], translation_vector[2])

        else:
            #self.cuprint( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None
 
    def execute(self, userdata):

        self.cuprint("executing")
        self.toggle_waypoint_control(True)

        self.drive_client.enable()
        self.depth_client.enable()
        self.strafe_client.enable()

        self.cuprint("servoing")

        r = rospy.Rate(self.rate)
        print("")
        while not rospy.is_shutdown():

            if self.offset is not None:

                drive_error = 1.0 - self.offset[2]
                self.drive_client.set_setpoint(self.drive_client.get_standard_state() - drive_error)

                strafe_error = self.offset[0]
                self.strafe_client.set_setpoint(self.strafe_client.get_standard_state() + strafe_error)

                depth_error = 0.07 - self.offset[1]
                depth_rate_limited = min(max(depth_error, -0.2), 0.2)
                self.depth_client.set_setpoint(self.depth_client.get_standard_state() + depth_rate_limited)

                error_string_drive = "Error drive: " + bcolors.HEADER + str(round(drive_error,2)) +bcolors.ENDC
                error_string_depth = " | depth: " + bcolors.HEADER + str(round(depth_error,2)) + bcolors.ENDC
                error_string_strafe = " | strafe: " + bcolors.HEADER + str(round(strafe_error,2)) + bcolors.ENDC
                self.cuprint(error_string_drive + error_string_depth + error_string_strafe, print_prev_line=True)

                if abs(strafe_error) < 0.025 and abs(drive_error) < 0.1 and abs(depth_error) < 0.025:
                    break

            rospy.sleep(0.1)

        self.actuator_service(2, 500)
    
        self.cuprint("slayed")
        self.drive_client.disable()
        self.depth_client.disable()
        self.strafe_client.disable()

        return "slayed"

class Backup(Objective):
    outcomes = ['backed_up','timed_out']

    def __init__(self, task_name):
        name = task_name + "/Backup"
        super(Backup, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/" + task_name.lower() + "/approach_stop_dist")

    def execute(self, userdata):
        self.cuprint("executing")
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.enable()
        self.drive_client.set_setpoint(cur_state - self.carrot_dist + 0.3) # add a little more so we hit the buoy
        rospy.sleep(5)
        self.toggle_waypoint_control(False)
        self.drive_client.disable()
        self.cuprint("backed up")
        return "backed_up"


class Retrace(Objective):
    outcomes = ['found','not_found']

    target_class_id = "dracula"

    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + u"/Rétrace".encode("utf-8"))
        self.listener = listener
        self.retrace_hit_cnt = rospy.get_param("tasks/" + task_name.lower() + "/retrace_hit_count")
        self.retrace_back_in_time = rospy.get_param("tasks/" + task_name.lower() + "/retrace_back_in_time")

    def execute(self, userdata):
        self.configure_darknet_cameras([1,1,1,1,1,0])
        self.toggle_waypoint_control(False)
        self.cuprint("executing")
        dobj_nums = self.listener.query_class(self.target_class_id)
        if not len(dobj_nums): #shouldn't be possible
            # do some timeout?
            userdata.outcome = "timed_out"
            return "not_found"

        #This is an assumption that we only have one dobj, which we *should*
        dobj_num = dobj_nums[0]

        # loop variables
        count = 0
        retraced_steps = 1
        first_bc = self.listener[dobj_num][-1]

        backup_time = first_bc.camera_header.stamp - rospy.Duration(self.retrace_back_in_time)
        
        breadcrumb_dvs = self.listener[dobj_num].get_dvectors_since_time(backup_time)

        len_dvec = len(breadcrumb_dvs)
        self.cuprint("breadcrumbs to follow since " + str(self.retrace_back_in_time) + " seconds ago:" + str(len_dvec))
        last_sub_pt = breadcrumb_dvs[len_dvec-retraced_steps].sub_pt 
        last_pose = Pose(last_sub_pt, self.cur_pose.orientation)

        # Start Retrace: Set first waypoint
        self.go_to_pose_non_blocking(last_pose, move_mode="strafe")
        # necessary for same-line printing
        print("")
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num) and count < self.retrace_hit_cnt: 
                #found object again
                count += 1
            elif count >= self.retrace_hit_cnt:
                self.cancel_way_client_goal()
                return "found"
            else:
                dist = self.get_distance(self.cur_pose.position, last_pose.position)
                self.cuprint("Breadcrumb #"+ bcolors.HEADER + str( retraced_steps)+ bcolors.ENDC+ " distance: " + bcolors.OKBLUE + str(dist) + bcolors.ENDC, print_prev_line=True)
                #self.cuprint("distance to goal: "+ str())
                if self.check_reached_pose(last_pose, 0.7):
                    self.cancel_way_client_goal()
                    retraced_steps += 1
                    if (retraced_steps > len_dvec):
                        #means we retraces all our steps. We're lost.
                        return "not_found"
                    # Goto Last dvector
                    # get last dvector sub_pt
                    last_sub_pt = breadcrumb_dvs[len_dvec-retraced_steps].sub_pt 
            
                    last_pose = Pose(last_sub_pt, self.cur_pose.orientation)
                    #set waypoint to this point. Give some distance to account for error in bearing and sub_pt
                    # Set waypoint
                    self.go_to_pose_non_blocking(last_pose, move_mode="strafe", log_print=False)

            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"

        #clean up if we are killed
        return "not_found"
        


