#!/usr/bin/env python

"""
Simulates hydrophones in Gazebo
- should publish measurements periodically
- if sub is facing w/in Psi degrees of direction of obstacle
--- meas noise ~N(0, sigma^2)
- else
--- meas noise ~N(u, sigma^2), with some offset u ~ U(-45,45)
- only one object at a time can be the source of the measurements
--- available objects on cusub_common/fakehydro/object_options
--- current object on cusub_common/fakehydro/active
--- configured via service cusub_common/fakehydro/configure_active
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import tf

class HydroFaker:
    name = "HydroFaker"
    def __init__(self):
        objects = rospy.get_param("fakehydro/objects")
        self.object_positions = {}
        for obj in objects.keys():
            self.object_positions[obj] = None
            rospy.Subscriber(objects[obj]["odom_topic"], Odometry, self.object_pose_callback)

        self.meas_freq = rospy.get_param("fakehydro/meas_freq")
        self.active_obj = rospy.get_param("fakehydro/starting_object")
        if self.active_obj == "random":
            self.active_obj = np.random.choice(self.object_positions.keys())
        self.print_pinger_status()

        rospy.Subscriber("/leviathan/description/pose_gt", Odometry, self.sub_pose_callback)

        # TODO timer callback for publishing measurement
        # noise model
        # - rosservice for configuration

    def object_pose_callback(self, msg):
        topic = msg._connection_header["topic"]
        for obj in self.object_positions.keys():
            if obj in topic:
                self.object_positions[obj] = msg
                break

    def sub_pose_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion([x,y,z,w])        
        self.position = msg.pose.pose.position

    # def run(self):
        # loop at freq
        # run 

    def print_pinger_status(self):
        self.smprint("active pinger set to " + bcolors.HEADER + self.active_obj + bcolors.ENDC)

    def smprint(self, string, warn=False):
        if warn:
            rospy.loginfo("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + bcolors.WARNING +"[WARN] "+ string + bcolors.ENDC)
        else:
            rospy.loginfo("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + string)

class bcolors: # For terminal colors
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

if __name__ == "__main__":
    rospy.init_node("fakehydro")
    h = HydroFaker()
    rospy.sleep()
    # h.run()