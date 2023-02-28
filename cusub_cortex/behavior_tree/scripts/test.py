#!/usr/bin/env python

import tasks.general as bt
import math_utilities as mu
import rospy
import time
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float64


class Test:

    root = bt.Sequence("root")
    counter = bt.Set("counter", "counter", 1, 1)
    cond_counter = bt.Condition("cond_counter", "counter", bt.blackboard["rate"] * 5)
    sequence = bt.Sequence("sequence")
    reset_counter = bt.Set("reset_counter", "counter", 0)
    velocity = bt.Set("velocity", "velocity", mu.Vector2(-1.0, 0.0), 2)

    root.nodes.append(counter)
    root.nodes.append(cond_counter)
    cond_counter.nodes.append(sequence)
    sequence.nodes.append(reset_counter)
    sequence.nodes.append(velocity)

    def create_publishers(self):
        pub1 = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/drive_vel/setpoint", Float64, queue_size=1)
        pub2 = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/strafe_vel/setpoint", Float64, queue_size=1)

        bt.blackboard["drive_publisher"] = pub1
        bt.blackboard["strafe_publisher"] = pub2

    def create_subscribers(self):
        rospy.Subscriber("move", Float64MultiArray, self.move_callback)

    def init_ros(self):
        rospy.init_node("behavior_tree")
        rate = rospy.Rate(bt.blackboard["rate"])

        while not rospy.is_shutdown():
            self.root.activate(bt.blackboard)

            # this updates the velocity of the robot from the blackboard variables
            # I put it here because it makes sense to run every time the tree runs
            if bt.blackboard["drive_publisher"] != None and bt.blackboard["strafe_publisher"] != None:
                bt.blackboard["drive_publisher"].publish(bt.blackboard["velocity"].x)
                bt.blackboard["strafe_publisher"].publish(-bt.blackboard["velocity"].y)

            rate.sleep()


    def move_callback(self, data):
        bt.blackboard["move"].x = data.data[0]
        bt.blackboard["move"].y = data.data[1]



test = Test()
test.create_publishers()
test.create_subscribers()
test.init_ros()