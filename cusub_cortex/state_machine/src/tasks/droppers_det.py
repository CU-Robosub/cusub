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
from geometry_msgs.msg import Pose, Point
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np

class Droppers(Task):
    name = "Droppers"

    def __init__(self):
        super(Droppers, self).__init__(self.name)

        # All Objectives share the same listener to gaurantee same data between objectives
        self.listener = DetectionListener() 

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive", "cusub_common/motor_controllers/elevAz_pid/")
        strafe_client = PIDClient(self.name, "strafe", "cusub_common/motor_controllers/elevAz_pid/")
        clients = {'drive_client' : drive_client, 'strafe_client' : strafe_client}
        search_classes = ["dropper_cover"]
        darknet_cameras = [1,1,0,0,1,1] # front 3 occams + downcam
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param(), darknet_cameras=darknet_cameras)
        self.approach = Approach(self.name, self.listener, clients)
        self.retrace = Retrace(self.name, self.listener)        

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']

    target_class_id = "dropper_cover"

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.listener = listener
        self.yaw_client = PIDClient(name, "yaw")
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]

        seconds = 4 #rospy.get_param("tasks/jiangshi/seconds_in_position")
        self.rate = 30 #rospy.get_param("tasks/jiangshi/new_dv_check_rate")
        self.count_target = seconds * self.rate
        self.count = 0
        
        self.pid_target = 0.0
        self.pid_target_thresh = 5
    
    def execute(self, userdata):
        self.cuprint("executing")

        # Find vampire_cute's dobject number and check for errors
        dobj_nums = self.listener.query_class(self.target_class_id)
        if len(dobj_nums) > 1: # Check if more than 1 instance of target_class
            self.cuprint(str(len(dobj_nums)) + " " + self.target_class_id + " classes detected!", warn=True)
            self.cuprint("selecting the first", warn=True)
        elif not dobj_nums: # Chck if target class is not present (shouldn't be possible)
            self.cuprint("somehow no " + self.target_class_id + " classes found?", warn=True)
            return "lost_object"
        dobj_num = dobj_nums[0]
        self.cuprint("located " + self.target_class_id + "'s dobject num: " + str(dobj_num))
        
        # TODO start a watch dog timer on detections

        # Enable the PID loops
        self.yaw_client.enable()
        self.yaw_client.set_setpoint(0.0)

        self.drive_client.enable()
        self.drive_client.set_setpoint(self.pid_target)
        self.strafe_client.enable()
        self.strafe_client.set_setpoint(self.pid_target)
        

        self.cuprint("servoing")
        printed = False
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.listener.check_new_dv(dobj_num):
                [az, el, mag] = self.listener.get_avg_bearing(dobj_num, num_dv=5)
                drive_state, strafe_state = self.transform_state(az, el)
                # print("---")
                # print("az: "+str(az) + "\tel:"+str(el))
                # print("drive_state: "+str(round(drive_state,2)) + "\strafe_state:"+str(round(strafe_state,2)))
                self.drive_client.set_state(drive_state)
                self.strafe_client.set_state(strafe_state)

                # self.depth_client.set_state(el * (1000/np.sqrt(mag))) # normalize the elevation bearing using the magnitude
                self.listener.clear_new_dv_flag(dobj_num)

                if self.check_in_position(drive_state, strafe_state):
                    self.count += 1
                    if self.count > self.count_target and not printed:
                        self.cuprint("In Position")
                        printed = True
                        # break
                else:
                    self.count = 0
                    printed = False
            if userdata.timeout_obj.timed_out:
                self.yaw_client.disable()
                self.drive_client.disable()
                self.depth_client.disable()
                userdata.outcome = "timed_out"
                return "not_found"

            self.drive_client.set_setpoint(self.pid_target, loop=False)
            self.strafe_client.set_setpoint(self.pid_target, loop=False)
            r.sleep()

        # self.yaw_client.disable()
        self.drive_client.disable()
        self.strafe_client.disable()
        return "in_position"

    def transform_state(self, az, elev):
        # Az between 0 and 2pi
        while az < 0:
            az += 2*np.pi
        
        drive_state = 100 * np.cos(elev)
        strafe_state = 100 * np.cos(elev)
        if az < np.pi / 2:
            strafe_state = abs(strafe_state)
            # print("quad 1")
        elif az < np.pi:
            strafe_state = abs(strafe_state)
            drive_state = -abs(drive_state)
            # print("quad 2")
        elif az < 3*np.pi/2:
            drive_state = -abs(drive_state)
            strafe_state = -abs(strafe_state)
            # print("quad 3")
        else:
            strafe_state = -abs(strafe_state)
            # print("quad 4")
        return drive_state, strafe_state

    def check_in_position(self, drive_state, strafe_state):
        drive_reached, strafe_reached = False, False
        if (drive_state - self.pid_target) < self.pid_target_thresh:
            drive_reached = True
        if (strafe_state - self.pid_target) < self.pid_target_thresh:
            strafe_reached = True
        return drive_reached and strafe_reached

class Slay(Objective):
    outcomes = ['slayed', 'timed_out']

    def __init__(self, task_name):
        name = task_name + "/Slay"
        super(Slay, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/jiangshi/slay_carrot_dist")

    def execute(self, userdata):
        self.cuprint("executing")
        self.wayToggle(False)
        self.drive_client.enable()
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.set_setpoint(cur_state + self.carrot_dist)
        rospy.sleep(10)
        self.cuprint("slayed")
        self.drive_client.disable()
        return "slayed"

class Backup(Objective):
    outcomes = ['backed_up','timed_out']

    def __init__(self, task_name):
        name = task_name + "/Backup"
        super(Backup, self).__init__(self.outcomes, name)
        self.drive_client = PIDClient(name, "drive")
        self.carrot_dist = rospy.get_param("tasks/jiangshi/slay_carrot_dist")

    def execute(self, userdata):
        self.cuprint("executing")
        cur_state = self.drive_client.get_standard_state()
        self.drive_client.enable()
        self.drive_client.set_setpoint(cur_state - self.slay_carrot_dist)
        rospy.sleep(10)
        self.wayToggle(True)
        self.drive_client.disable()
        self.cuprint("backed up")
        return "backed_up"


# TODO
class Retrace(Objective):
    outcomes = ['found','not_found']

    #TODO: confirm id names
    target_class_ids = ["dropper", "wolf", "bat"]


    def __init__(self, task_name, listener):
        super(Retrace, self).__init__(self.outcomes, task_name + "/Retrace")
        self.listener = listener
        #TODO: add param
        self.retrace_hit_cnt = rospy.get_param("tasks/dropper/retrace_hit_count")

    #TODO: Analyze assumption made: take most recent dvector from each target_class?
    # Or, go through each one in every dvector list?
    def find_nearest_breadcrumb(self, d_vectors):
        min_dis = float("Inf")
        index = 0
        for i in range(len(d_vectors)):
            new_dis = self.get_distance(self.cur_pose.position, d_vectors[i].sub_pt)
            if new_dis < min_dis:
                min_dis = new_dis
                index = i
        return index

    def execute(self, userdata):
        self.smprint("executing R`etrace")
        dobj_dict = self.listener.query_classes(self.target_class_ids)
        # For droppers, there should only be 3 Dobjects: droppers, wolf, bat.
        # So now I want to extract these Dobjects from the dobj_dict,
        # while hopefully adding some false positive rejection...
        dobj_nums = []
        for target in self.target_class_ids:
            nums = dobj_dict[target]
            if len(nums) > 1:
                # shouldn't have multiple dobjects, choose the one with most dvectors
                max_num = max([len(self.listener.dobjects[i].dvectors) for i in nums])
                dobj_nums.append(self.listener.dobjects.index(max_num))
            else:
                dobj_nums.append(nums[0])

        #now, dobj_nums has the object index for each of the three Dobjects, if they exist    
        #loop variables
        count = 0
        retraced_steps = [1 for i in self.target_class_ids]
        len_dvecs = [len(self.listener.dobjects[id].dvectors) for id in self.target_class_ids]

        recent_dvectors = [self.listener.dobjects[id].get_d(len_dvecs-retraced_steps[id]) for id in self.target_class_ids]
        i = self.find_nearest_breadcrumb(recent_dvectors)
        last_sub_pt = recent_dvectors[i].sub_pt
        last_pose = Pose(last_sub_pt, self.cur_pose.orientation)

        #Start Retrace: Set first waypoint
        self.go_to_pose_non_blocking(last_pose, userdata.timeout_obj, move_mode="strafe")
        while not rospy.is_shutdown():
            if self.listener.check_new_dvs(dobj_nums) != -1 and count < self.retrace_hit_cnt: 
                #found object again
                count += 1
                if count >= self.retrace_hit_cnt:
                    self.cancel_way_client_goal()
                    return "found"
            else:
                if self.check_reached_pose(last_pose):
                    retraced_steps[i] += 1
                    if (retraced_steps[i] > len_dvecs[i]):
                        self.target_class_ids.pop(i)     # class list is local to this state so pop() is ok?
                        if len(self.target_class_ids) == 0:
                            #means we retraces all our steps. We're lost.
                            return "not_found"
                    # Goto Last dvector
                    # get last dvector sub_pt
                    recent_dvectors = [self.listener.dobjects[id].get_d(len_dvecs-retraced_steps[id]) for id in self.target_class_ids]
                    i = self.find_nearest_breadcrumb(recent_dvectors)
                    last_sub_pt = recent_dvectors[i].sub_pt
                    last_pose = Pose(last_sub_pt, self.cur_pose.orientation)
            
                    #set waypoint to this point. Give some distance to account for error in bearing and sub_pt
                    # Set waypoint
                    self.go_to_pose_non_blocking(last_pose,userdata.timeout_obj,move_mode="strafe")

            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"
            rospy.sleep(.5)

        #clean up if we are killed
        return "not_found"