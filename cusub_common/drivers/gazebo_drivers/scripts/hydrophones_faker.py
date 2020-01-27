#!/usr/bin/env python
from __future__ import division
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
from gazebo_drivers.msg import Hydrophones
from geometry_msgs.msg import PoseStamped
import tf

class HydroFaker:
    name = "HydroFaker"
    def __init__(self):
        self.cuprint("loading")
        objects = rospy.get_param("fakehydro/objects")
        self.object_positions = {}
        for obj in objects.keys():
            self.object_positions[obj] = None
            rospy.Subscriber(objects[obj]["odom_topic"], Odometry, self.object_pose_callback)

        self.psi = (np.pi / 180.0) * rospy.get_param("fakehydro/psi_proximity_degrees")
        self.sigma = (np.pi / 180.0) * rospy.get_param("fakehydro/standard_deviation")
        meas_freq = rospy.get_param("fakehydro/meas_freq")
        self.active_obj = rospy.get_param("fakehydro/starting_object")
        self.seq = 0
        if self.active_obj == "random":
            self.active_obj = np.random.choice(self.object_positions.keys())
        self.print_pinger_status()

        self.debug_pub = rospy.Publisher("hydrophones/debug_measurements", PoseStamped, queue_size=10)
        self.pub = rospy.Publisher("hydrophones/measurements", Hydrophones, queue_size=10)
        rospy.Subscriber("/leviathan/description/pose_gt", Odometry, self.sub_pose_callback)

        rospy.Timer(rospy.Duration(1 / meas_freq), self.publish_measurement)
        
        # TODO set sub starting coords to [35.3, 8.72, -2.21]
        # TODO preserve the bias we produce until, we turn by more than 30 degrees OR we move more than 2m
        # TODO rosservice for configuration

    def publish_measurement(self, msg):
        self.cuprint("publishing")
        self.seq += 1
        obj_position = self.object_positions[self.active_obj]
        if obj_position == None:
            self.cuprint("haven't received " + obj_position + "'s pose_gt", warn=True)
            return

        xdiff = obj_position.x - self.position.x
        ydiff = obj_position.y - self.position.y
        facing_angle = np.arctan2(ydiff, xdiff)
        diff_1 = self.normalize_angle_1(facing_angle) - self.normalize_angle_1(self.yaw)
        diff_2 = self.normalize_angle_2(facing_angle) - self.normalize_angle_2(self.yaw)
        min_angle_index = np.argmin([abs(diff_1), abs(diff_2)])
        angle_diff = [diff_1, diff_2][min_angle_index]

        msg = Hydrophones()
        msg.header.frame_id = "leviathan/description/base_link"
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq = self.seq
        if abs(angle_diff) < self.psi:
            msg.azimuth = np.random.normal(angle_diff, self.sigma)
        else:
            self.cuprint("...biased")
            random_bias = np.random.uniform(self.psi, - self.psi)
            msg.azimuth = np.random.normal(random_bias, self.sigma)
        self.pub.publish(msg)

        # Debug pub
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position = self.position
        quat = tf.transformations.quaternion_from_euler(0,0, msg.azimuth)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.debug_pub.publish(pose_msg)

    """ Normalizes -pi,pi """
    def normalize_angle_1(self, angle):
        return np.mod(angle + np.pi, 2*np.pi ) - np.pi
    
    """ Normalizes 0,2pi """
    def normalize_angle_2(self, angle):
        while angle < 0:
            angle += 2*np.pi
        while angle > 2*np.pi:
            angle -= 2*np.pi
        return angle

    def object_pose_callback(self, msg):
        topic = msg._connection_header["topic"]
        for obj in self.object_positions.keys():
            if obj in topic:
                self.object_positions[obj] = msg.pose.pose.position
                break

    def sub_pose_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion([x,y,z,w])
        self.position = msg.pose.pose.position

    def print_pinger_status(self):
        self.cuprint("active pinger set to " + bcolors.HEADER + self.active_obj + bcolors.ENDC)

    def cuprint(self, string, warn=False):
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
    rospy.spin()