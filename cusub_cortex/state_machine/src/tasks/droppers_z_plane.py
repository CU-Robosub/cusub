#!/usr/bin/env python
from __future__ import division
"""
Droppers Task, attempts to drop 2 markers in the bin
Objectives:
- Search
- Approach
"""
from tasks.task import Task, Objective
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from localizer.msg import Detection
from actuator.srv import ActivateActuator

class Droppers(Task):
    name = "Droppers"

    def __init__(self):
        super(Droppers, self).__init__(self.name)

        # All Objectives share the same listener to gaurantee same data between objectives
        self.listener = DetectionListener() 

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive")
        strafe_client = PIDClient(self.name, "strafe")
        clients = {'drive_client' : drive_client, 'strafe_client' : strafe_client}
        search_classes = ["dropper_cover", "wolf"]
        darknet_cameras = [1,1,0,0,1,1] # front 3 occams + downcam
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param(), darknet_cameras=darknet_cameras)
        self.approach = Approach(self.name, clients)
        self.drop = Drop(self.name, clients)
        self.revisit = Revisit(self.name, self.listener)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Revisit', self.revisit, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'Drop', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Drop', self.drop, transitions={'dropped':'manager', 'timed_out':'manager', 'lost_object':'Revisit'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']
    
    target_class = "dropper_cover"

    def __init__(self, task_name, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]

        self.xy_distance_thresh = rospy.get_param("tasks/droppers/xy_dist_thresh_app")

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

        # Check we've got a pose, if not return to lost_object which will return to pose of the detection
        if self.dropper_pose == None:
            rospy.sleep(2) # Wait for the pose to be sent by localizer
            if self.dropper_pose == None:
                return "lost_object"

        self.toggle_waypoint_control(True)
        self.drive_client.enable()
        self.strafe_client.enable()

        drive, strafe = self.get_relative_drive_strafe(self.dropper_pose)
        self.clear_new_pose_flag()
        drive_setpoint = self.drive_client.get_standard_state() + drive
        strafe_setpoint = self.strafe_client.get_standard_state() + strafe
        self.drive_client.set_setpoint(drive_setpoint)
        self.strafe_client.set_setpoint(strafe_setpoint)

        self.cuprint("servoing")
        printed = False
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.check_new_pose():
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
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "not_found"

            self.drive_client.set_setpoint(drive_setpoint, loop=False)
            self.strafe_client.set_setpoint(strafe_setpoint, loop=False)
            r.sleep()
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
        x_diff = self.dropper_pose.position.x - self.cur_pose.position.x
        y_diff = self.dropper_pose.position.y - self.cur_pose.position.y
        return np.linalg.norm([x_diff, y_diff]) < self.xy_distance_thresh

    def dropper_pose_callback(self, msg):
        if msg.class_id == self.target_class:
            self.dropper_pose = msg.pose.pose
            self.new_pose_flag = True

    def clear_new_pose_flag(self):
        self.new_pose_flag = False
    def check_new_pose(self):
        return self.new_pose_flag

class Drop(Approach): # share that code again...
    outcomes = ['dropped','timed_out', 'lost_object']

    target_class = "wolf"
    
    def __init__(self, task_name, clients):
        name = task_name + "/Drop"
        super(Approach, self).__init__(self.outcomes, name)
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.depth_client = PIDClient(name, "depth")

        self.xy_distance_thresh = rospy.get_param("tasks/droppers/xy_dist_thresh_drop")
        self.drop_depth = rospy.get_param("tasks/droppers/drop_depth")

        self.rate = 30
        self.count_target = 0 # 0 since once we're in position, drop
        self.count = 0

        self.dropper_pose = None
        self.new_pose_flag = False
        # rospy.Subscriber("cusub_cortex/mapper_out/start_gate", PoseStamped, self.dropper_pose_callback) # mapper
        rospy.Subscriber("cusub_perception/mapper/task_poses", Detection, self.dropper_pose_callback)

        self.cuprint("waiting for actuator service")
        rospy.wait_for_service("cusub_common/activateActuator")
        self.cuprint("...connected")
        self.actuator_service = rospy.ServiceProxy("cusub_common/activateActuator", ActivateActuator)

    def execute(self, userdata):
        self.cuprint("diving")
        self.depth_client.set_setpoint(self.drop_depth)
        rospy.sleep(1.5)

        ret = super(Drop, self).execute(userdata)
        if ret == "in_position":
            self.cuprint("Bombs Away")
            self.actuate_dropper(1)
            # self.actuate_dropper(2)
            return "dropped"
        else:
            return ret

    def actuate_dropper(self, dropper_num):
        self.actuator_service(dropper_num, 500)
# TODO
class Revisit(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener):
        super(Revisit, self).__init__(self.outcomes, task_name + "/Revisit")