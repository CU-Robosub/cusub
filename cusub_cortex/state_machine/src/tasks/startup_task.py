#!/usr/bin/env python
from __future__ import division
"""
Startup Task, allows the person on the competition dock time to remove the tether from the vehicle before it starts its autonomous run.
Waits briefly at the surface before diving.
"""
import rospy
import smach
import smach_ros
import rospy
from std_msgs.msg import Float64
from tasks.task import Task, Objective

class Startup(Task):
    name = "Startup"

    def __init__(self):
        super(Startup, self).__init__(self.name) # become a state machine first
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.dive = Dive(self.name)

    def link_objectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('Dive', self.dive, transitions={'done':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Dive(Objective):

    outcomes=['done']

    def __init__(self, task_name):
        name = task_name + "/Dive"
        super(Dive, self).__init__(self.outcomes, name)
        self.dive_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)

    def execute(self, userdata):
        depth_msg = Float64()
        depth_msg.data = 0.2 # experimentally determined from leviathan
        self.cuprint("PULL TETHER")
        while not rospy.is_shutdown():
            self.dive_pub.publish(depth_msg)
            if userdata.timeout_obj.timed_out:
                break
            rospy.sleep(0.25)
        depth_msg.data = rospy.get_param("tasks/startup/depth")
        self.cuprint("diving")
        userdata.timeout_obj.set_new_time(rospy.get_param("tasks/startup/dive_time"))
        while not rospy.is_shutdown():
            self.dive_pub.publish(depth_msg)
            if userdata.timeout_obj.timed_out:
                break
            rospy.sleep(0.25)
        userdata.outcome = "success"
        return "done"
