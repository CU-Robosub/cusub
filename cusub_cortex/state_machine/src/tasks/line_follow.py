#!/usr/bin/env python
from __future__ import division
"""
Startup Task, allows the man on the competition dock time to remove the tether from the vehicle before it starts its autonomous run.
Waits briefly at the surface before diving.
"""
import rospy
import smach
import smach_ros
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64, String
from tasks.task import Task, Objective
from tasks.pid_client import PIDClient

class LineFollow(Task):
    name = "LineFollow"

    def __init__(self):
        super(LineFollow, self).__init__(self.name) # become a state machine first
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        drive_client = PIDClient(self.name, "drive")
        strafe_client = PIDClient(self.name, "strafe")
        depth_client = PIDClient(self.name, "depth")
        yaw_client = PIDClient(self.name, "yaw")
        clients = {
            "drive_client": drive_client,
            "strafe_client": strafe_client,
            "depth_client": depth_client,
            "yaw_client": yaw_client
        }
        self.cuprint("1")
        self.line_follow = GoLineFollow(self.name, clients)
        self.cuprint("2")

    def link_objectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('GoLineFollow', self.line_follow, transitions={'done':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class GoLineFollow(Objective):

    outcomes=['done']

    def __init__(self, task_name, clients):
        name = task_name + "/LineFollow"
        super(GoLineFollow, self).__init__(self.outcomes, name)
        self.cuprint("initializing")
        self.drive_client = clients["drive_client"]
        self.strafe_client = clients["strafe_client"]
        self.depth_client = clients["depth_client"]
        self.yaw_client = clients["yaw_client"]
        self.initialized = False
        self.depth_drop = 0.5
        self.max_drive_setpoint = 1.0
        rospy.Subscriber("line_follow_control", String, self.line_follow_control)

    def execute(self, userdata):
        self.PID_enable()
        # MOVE_MODE = "yaw"
        self.initialized = True
        self.depth_setpoint = self.depth_client.get_standard_state()
        self.drive_setpoint = 0.0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

        userdata.outcome = "success"
        return "done"

    def PID_enable(self):
        self.drive_client.enable()
        self.strafe_client.enable()
        self.depth_client.enable()
        self.yaw_client.enable()

    def PID_disable(self):
        self.drive_client.disable()
        self.strafe_client.disable()
        self.depth_client.disable()
        self.yaw_client.disable()

    def line_follow_control(self, msg):
        if not self.initialized:
            return

        if msg.data == "dive":
            self.cuprint("diving")
            self.depth_setpoint = self.depth_setpoint - self.depth_drop
            self.drive_setpoint = 0.0
            self.PID_all_set_setpoint(self.drive_setpoint, self.depth_setpoint)
            return

        if msg.data == "forward":
            self.cuprint("forward")
            self.depth_setpoint = self.depth_client.get_standard_state()
            self.drive_setpoint = self.max_drive_setpoint
            self.PID_all_set_setpoint(self.drive_setpoint, self.depth_setpoint)
            return

        if msg.data == "backward":
            self.cuprint("backward")
            self.depth_setpoint = self.depth_client.get_standard_state()
            self.drive_setpoint = -1 * self.max_drive_setpoint
            self.PID_all_set_setpoint(self.drive_setpoint, self.depth_setpoint)
            return

        if msg.data == "stop":
            self.cuprint("stop")
            self.depth_setpoint = self.depth_client.get_standard_state()
            self.drive_setpoint = 0.0
            self.PID_all_set_setpoint(self.drive_setpoint, self.depth_setpoint)
            return

        self.cuprint(msg.data + " is not supported")

        

    def PID_all_set_setpoint(self, drive_sp, depth_sp):
        self.drive_client.set_setpoint(drive_sp, loop=False)
        self.depth_client.set_setpoint(depth_sp, loop=False)
        
