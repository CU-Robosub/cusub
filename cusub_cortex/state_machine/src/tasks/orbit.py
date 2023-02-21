#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from tasks.task import Task, Objective, Timeout
from tasks.search import Search
from tasks.pid_client import PIDClient
import rospy, smach
from detection_listener.listener import DetectionListener
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


## TO-DO - add params in missionconfig

class Orbit(Task):
    name = "Orbit"

    def __init__(self):
        super(Orbit, self).__init__(self.name)

        self.listener = DetectionListener()

        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive")
        strafe_client = PIDClient(self.name, "strafe")
        yaw_client = PIDClient(self.name, "yaw")
        clients = {'drive_client' : drive_client, 'strafe_client' : strafe_client, 'yaw_client' : yaw_client}
        search_classes = ["dropper_cover", "wolf"]
        darknet_cameras = [0,0,0,0,0,1] # front 3 occams + downcam
        self.search = Search(self.name, self.listener, search_classes, self.get_prior_param(), darknet_cameras=darknet_cameras)
        self.approach = Approach(self.name, self.listener, clients)
        self.revolve = Revolve(self.name, self.listener, clients) 

    def link_objectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Approach', self.approach, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timout_obj', 'outcome':'outcome'})
            smach.Revolve.add('Revolve', self.revolve, transitions={'found':'Approach', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timout_obj', 'outcome':'outcome'})

class Approach(Objective):
    outcomes = ['in_position','timed_out', 'lost_object']
    
    target_class_ids = ["dropper_cover", "wolf"]

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Approach"
        super(Approach, self).__init__(self.outcomes, name)
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.listener = listener

        # approach via any target class id
        # focus on the priority id onoce we detect it.
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
        rospy.Subscriber("cusub_perception/mapper/task_poses", Detection, self.dropper_pose_callback) #detection is an obj, adds studff so query class can use it
        
    def execute(self, userdata):
        self.cuprint("executing")
        self.configure_darknet_cameras([0,0,0,0,0,1])
        self.toggle_waypoint_control(True)

        # Find dropper_cover's dobject number and check for errors
        dobj_dict = self.listener.query_classes(self.target_class_ids)  #dobj is detected object
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
                    exit() ## just added this @xavier
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

class Revolve(Objective):
    outcomes = ['start', 'finish', 'lost_object', 'confused']
    target_class_ids = ["dropper_cover", "wolf"]

    def __init__(self, task_name, listener, clients):
        name = task_name + "/Revolve"
        super(Revolve, self).__init__ 

# i think i can use that one fxn to get azimuth, then i can probably use our dobj as the reference??