#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function
"""
Orbit task made to approach an object, stay a given distance away
from it, then circle it once before terminating.
- Search
- Approach
- Slay
- Back Up
"""
from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose
from localizer_msgs.msg import Detection
from actuator.srv import ActivateActuator
from cusub_print.cuprint import bcolors
from geometry_msgs.msg import Pose, Point, Quaternion

class orbitObject(Task):
    def __init__(self):
        super(orbitObject, self).__init__(self.name)

        # All Objectives share the same listener to gaurantee same data between objectives
        self.listener = DetectionListener() 

        self.init_objectives()
        self.link_objectives()
    def init_objectives(self):
        self.Approach(self.name)
        return super().init_objectives()
    def link_objectives(self):
        return super().link_objectives()
    # task is the parent class of of objective, objective has children classes.
    # super keyword indicates the creation of a parent object from the child class
    

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']
    
    target_class_ids = ["box", "red_button"]

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.listener = listener

        # approach via any target class id
        # focus on the priority id once we detect it.
        self.priority_id_flag = False
        self.priority_class_id = "wolf"

        self.xy_distance_thresh = rospy.get_param("tasks/droppers/xy_dist_thresh_app")

        self.retrace_timeout = rospy.get_param("tasks/" + task_name.lower() + "/retrace_timeout", 2)
        seconds = rospy.get_param("tasks/droppers/seconds_in_position")
        self.rate = 30
        self.count_target = seconds * self.rate
        self.count = 0

        self.dropper_pose = None
        self.new_pose_flag = False
        # rospy.Subscriber("cusub_cortex/mapper_out/start_gate", PoseStamped, self.dropper_pose_callback) # mapper
        rospy.Subscriber("cusub_perception/mapper/task_poses", Detection, self.dropper_pose_callback)
        
    def execute(self, userdata):
        self.cuprint("executing")
        self.configure_darknet_cameras([0,0,0,0,0,1])
        self.toggle_waypoint_control(True)

        # Find dropper_cover's dobject number and check for errors
        dobj_dict = self.listener.query_classes(self.target_class_ids)
        if not dobj_dict: # Check if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + str(self.target_class_ids) + " classes found in listener?", warn=True)
            return "lost_object"
        print_str = "dobj nums found: "
        for class_ in dobj_dict:
            print_str += bcolors.HEADER + bcolors.BOLD + class_ + ": " + bcolors.ENDC + str(dobj_dict[class_][0]) + bcolors.ENDC + "; "
        self.cuprint(print_str)

        watchdog_timer = Timeout(self.name + u"/Rétrace Watchdog".encode("utf-8"))

        # Check we've got a pose, if not return to lost_object which will return to pose of the detection
        if self.dropper_pose == None:
            rospy.sleep(2) # Wait for the pose to be sent by localizer
            if self.dropper_pose == None:
                self.cuprint("Dropper Pose still not received. ", warn=True)
                self.priority_id_flag = False
                return "lost_object"

        self.drive_client.enable()
        self.strafe_client.enable()

        drive, strafe = self.get_relative_drive_strafe(self.dropper_pose)
        self.clear_new_pose_flag()
        drive_setpoint = self.drive_client.get_standard_state() + drive
        strafe_setpoint = self.strafe_client.get_standard_state() + strafe
        self.drive_client.set_setpoint(drive_setpoint)
        self.strafe_client.set_setpoint(strafe_setpoint)
        watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

        self.cuprint("servoing")
        printed = False
        r = rospy.Rate(self.rate)
        print("") # overwritten by servoing status
        while not rospy.is_shutdown():
            if self.check_new_pose():
                watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)
                self.clear_new_pose_flag()
                drive, strafe = self.get_relative_drive_strafe(self.dropper_pose)
                drive_setpoint = self.drive_client.get_standard_state() + drive
                strafe_setpoint = self.strafe_client.get_standard_state() + strafe

                if self.check_in_position():
                    self.count += 1
                    if self.count > self.count_target and not printed:
                        printed = True
                        break
                else:
                    self.count = 0
                    printed = False

            elif watchdog_timer.timed_out:
                self.drive_client.disable()
                self.strafe_client.disable()
                self.cuprint("Retrace watchdog timed out. :(", warn=True)
                self.priority_id_flag = False
                return "lost_object"

            if userdata.timeout_obj.timed_out:
                watchdog_timer.timer.shutdown()
                userdata.outcome = "timed_out"
                return "not_found"

            self.drive_client.set_setpoint(drive_setpoint, loop=False)
            self.strafe_client.set_setpoint(strafe_setpoint, loop=False)
            r.sleep()
        watchdog_timer.timer.shutdown()
        return "in_position"

    def get_relative_drive_strafe(self, dropper_pose):
        """
        Gets the relative drive, strafe changes to reach the dropper pose
        """
        ori = self.cur_pose.orientation
        sub_rol, sub_pitch, sub_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        x_diff = dropper_pose.position.x - self.cur_pose.position.x
        y_diff = dropper_pose.position.y - self.cur_pose.position.y

        # Numerical stability
        if abs(x_diff) < 0.0001:
            x_diff = 0.0001 * np.sign(x_diff)
        if abs(y_diff) < 0.0001:
            y_diff = 0.0001 * np.sign(y_diff)

        dist_to_dropper = np.linalg.norm([x_diff, y_diff])
        relative_yaw_diff = np.arctan2(y_diff, x_diff) - sub_yaw
        drive = dist_to_dropper * np.cos(relative_yaw_diff)
        strafe = - dist_to_dropper * np.sin(relative_yaw_diff) # flip strafe

        return [drive, strafe]

    def check_in_position(self): 
        x_diff = round( self.dropper_pose.position.x - self.cur_pose.position.x, 2)
        y_diff = round( self.dropper_pose.position.y - self.cur_pose.position.y, 2)
        x_str = "{:.2f}".format(x_diff)
        y_str = "{:.2f}".format(y_diff)
        self.cuprint("Error x: " + bcolors.HEADER + x_str + bcolors.ENDC + " | y: " + bcolors.HEADER + y_str + bcolors.ENDC, print_prev_line=True)
        return np.linalg.norm([x_diff, y_diff]) < self.xy_distance_thresh

    def dropper_pose_callback(self, msg):
        if msg.class_id in self.target_class_ids and not self.priority_id_flag:
            if msg.class_id == self.priority_class_id:
                self.priority_id_flag = True
            self.dropper_pose = msg.pose.pose
            self.new_pose_flag = True
        elif self.priority_id_flag:
            if msg.class_id == self.priority_class_id:
                self.dropper_pose = msg.pose.pose
                self.new_pose_flag = True

    def clear_new_pose_flag(self):
        self.new_pose_flag = False
    def check_new_pose(self):
        return self.new_pose_flag

class Orbit(Objective):
    def __init__(self, outcomes, objtv_name):
        super(Orbit, self).__init__(outcomes, objtv_name)
        
    def execute(self, ud):
        watchdog_timer = Timeout(self.name + u"/Rétrace Watchdog".encode("utf-8"))
        
        drive, strafe = self.get_relative_drive_strafe(self.dropper_pose)
        self.clear_new_pose_flag()
        drive_setpoint = self.drive_client.get_standard_state() + drive
        strafe_setpoint = self.strafe_client.get_standard_state() + strafe
        self.drive_client.set_setpoint(drive_setpoint)
        self.strafe_client.set_setpoint(strafe_setpoint)
        watchdog_timer.set_new_time(self.retrace_timeout, print_new_time=False)

        return super().execute(ud)
    # function returns the current strafe and drive.
    def get_relative_drive_strafe(self, dropper_pose):
        """
        Gets the relative drive, strafe changes to reach the dropper pose
        """
        ori = self.cur_pose.orientation
        sub_rol, sub_pitch, sub_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        x_diff = dropper_pose.position.x - self.cur_pose.position.x
        y_diff = dropper_pose.position.y - self.cur_pose.position.y

        # Numerical stability
        if abs(x_diff) < 0.0001:
            x_diff = 0.0001 * np.sign(x_diff)
        if abs(y_diff) < 0.0001:
            y_diff = 0.0001 * np.sign(y_diff)

        dist_to_dropper = np.linalg.norm([x_diff, y_diff])
        relative_yaw_diff = np.arctan2(y_diff, x_diff) - sub_yaw
        drive = dist_to_dropper * np.cos(relative_yaw_diff)
        strafe = - dist_to_dropper * np.sin(relative_yaw_diff) # flip strafe

        return [drive, strafe]
    # to change whether the sub goes left or right, you need to change the self.strafe_client.set_setpoint() parameter
    # to a higher or lower number than what the current state is. Need to somehow get the current strafe state number,
    # figure out if higher is right or lower.
    # In context with the rotating problem, need to change the yah of the sub and strafe to the right to complete a circle.
    # figure out how to set the yah orientation for the sub. Try making a yah client to control through a pid loop just like
    # strafe and drive functions.
    def orbitObject(self, dropper_pose, radius)
        self.target_pose = dropper_pose
        self.target_pose.position.x = dropper_pose.position.x - radius
        self.drive_client.get_relative_drive_strafe(self, target_pose)
        self.strafe_client.get_relative_drive_strage(self, target_pose)
        
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