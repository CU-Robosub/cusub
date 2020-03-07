from __future__ import division
import rospy
from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.retrace import Retrace
from geometry_msgs.msg import Pose
from cusub_print.cuprint import bcolors

"""
Torpedoes Task, attempts to 
Objectives:
- Search
- Approach
"""
from tasks.pid_client import PIDClient
import rospy
import smach
import smach_ros
from detection_listener.listener import DetectionListener
import numpy as np
from localizer.msg import Detection
from actuator.srv import ActivateActuator


class Torpedoes(Task):
    name = "torpedoes"

    def __init__(self):
        super(Torpedoes, self).__init__(self.name)

        # All Objectives share the same listener to guarantee same data between objectives
        self.listener = DetectionListener() 

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive")
        strafe_client = PIDClient(self.name, "strafe")
        depth_client = PIDClient(self.name, "depth")
        clients = {'drive_client' : drive_client, 'strafe_client' : strafe_client, 'depth_client' : depth_client}
        search_classes = ["dracula", "heart", "open_oval"]
        # search_classes = ["jiangshi", "button"]
        darknet_cameras = [1,1,0,0,1,0] # front 3 occams
        self.retrace = Retrace(self.name, self.listener, search_classes)                
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param(), darknet_cameras=darknet_cameras)
        self.approach = Approach(self.name, self.listener, clients, self.retrace.watchdog_timer, darknet_cameras=darknet_cameras)
        self.shoot = Shoot(self.name, self.listener, clients)

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Retrace', self.retrace, transitions={'found':'Approach', 'not_found':'Search'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'in_position':'Drop', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Drop', self.drop, transitions={'dropped':'manager', 'timed_out':'manager', 'lost_object':'Retrace'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Approach(Objective):
    outcomes = ['in position', 'timed_out', 'lost_object']

    def __init__(self, task_name, listener, clients, target_classes, darknet_cameras=[1,0,0,1,1,0]):
        name = task_name + "/Approach"
        super(Approach, self).__init(self.outcomes, name)
        self.drive_client = clients['drive_client']
        self.strafe_client = clients['drive_client']
        self.depth_client = clients['drive_client']
        self.listener = listener
        self.darknet_cameras = darknet_cameras

        self.priority_id_flag = False
        self.priority_class_id = "heart"

        self.xy_distance_thresh = rospy.get_param("tasks/" + name + "/xy_dist_thresh_app")

        self.retrace_timeout = rospy.get_param("tasks/" + name + "/retrace_timeout")
        

    def execute(self, userdata):
        self.cuprint("executing")
        self.configure_darknet_cameras(self.darknet_cameras)


        dobj_dict = self.listener.query_classes(self.target_classes)
        self.cuprint(srt(dobj_dict))
        if not dobj_dict:
            self.cuprint("somehow no detections...?", warn=True)
            return "lost_object"
        print_str = "dobj nums found: "
        for class_ in dobj_dict:
            print_str += bcolors.HEADER + bcolors.BOLD + class_ + ": " + bcolors.ENDC + str(dobj_dict[class_][0]) + bcolors.ENDC + "; "
        self.cuprint(print_str)
        
        watchdog_timer = Timeout(u"Rétrace Watchdog Timer".encode("utf-8"))


class HoldTheBreath(Objective):
    # config to use torpedo cam
    # configure darknet cameras([0,0,0,0,0,0,1])

    #nudge control on the torp cam. 
    pass


class TakeEmOut(Objective):
    # call actuator service
    pass
