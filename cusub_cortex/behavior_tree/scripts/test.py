#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

from tasks.general import *
from tasks.basic import *

class Test:

    root = Sequence("root")
    counter = Set("counter", "counter", 1, "+")
    cond_counter = Condition("cond_counter", "counter", blackboard["rate"] * 5)
    sequence = Sequence("sequence")
    reset_counter = Set("reset_counter", "counter", 0)
    velocity = Set("velocity", "velocity", Vector2(-1.0, 0.0), "*")
    depth = Set("depth", "depth", 0.1, "-")
    yaw = Set("yaw", "yaw", 0.1, "+")

    root.nodes.append(counter)
    root.nodes.append(cond_counter)
    cond_counter.nodes.append(sequence)
    sequence.nodes.append(reset_counter)
    sequence.nodes.append(velocity)
    sequence.nodes.append(depth)

    def create_publishers(self):
        blackboard["drive_publisher"] = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/drive_vel/setpoint", Float64, queue_size=1)
        blackboard["strafe_publisher"] = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/strafe_vel/setpoint", Float64, queue_size=1)
        blackboard["depth_publisher"] = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        blackboard["yaw_publisher"] = rospy.Publisher("/leviathan/cusub_common/motor_controllers/pid/yaw/setpoint", Float64, queue_size=1)

    def create_subscribers(self):
        rospy.Subscriber("move", Float64MultiArray, self.move_callback)

    def init_ros(self):
        rospy.init_node("behavior_tree")
        rate = rospy.Rate(blackboard["rate"])

        while not rospy.is_shutdown():
            self.root.activate()

            # this updates the velocity of the robot from the blackboard variables
            # I put it here because it makes sense to run every time the tree runs
            if blackboard["drive_publisher"] != None and blackboard["strafe_publisher"] != None:
                blackboard["drive_publisher"].publish(blackboard["velocity"].x)
                blackboard["strafe_publisher"].publish(-blackboard["velocity"].y)
                blackboard["depth_publisher"].publish(blackboard["depth"])
                blackboard["yaw_publisher"].publish(blackboard["yaw"])

            rate.sleep()


    def move_callback(self, data):
        blackboard["velocity"].x = data.data[0]
        blackboard["velocity"].y = data.data[1]
        blackboard["depth"] = data.data[2]
        blackboard["yaw"] = data.data[3]



test = Test()
test.create_publishers()
test.create_subscribers()
test.init_ros()