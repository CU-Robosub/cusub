#!/usr/bin/env python
from __future__ import division
"""
Simulates hydrophones in Gazebo
- should publish measurements periodically
- if sub is facing w/in Psi degrees of direction of obstacle
--- meas noise ~N(0, sigma^2)
- else
--- meas noise ~N(u, sigma^2), with some offset u ~ U(-45,45)
--- offset is preserved until sub moves position_delta meters away from that position
--- or sub turns yaw_delta meters away from where the offset was calculated
------ this simulates the random fluctuations of the pinger signal at each position in transdec
- only one object at a time can be the source of the measurements
--- available objects on: cusub_common/hydrophones/debug_pinger_options
--- current object on: cusub_common/hydrophones/active
--- configured via service: cusub_common/hydrophones/switch_active_pinger
--- check whether the noise is biased: cusub_common/hydrophones/debug_is_biased
--- a pose msg for viewing in rviz is publish on: cusub_common/hydrophones/debug_measurements
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from gazebo_drivers.msg import Hydrophones, PingerOptions
from gazebo_drivers.srv import PingerSwitch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
import tf
from cusub_print.cuprint import CUPrint, bcolors

class HydroFaker:
    name = "HydroFaker"
    def __init__(self):
        self.cuprint = CUPrint(self.name)
        self.cuprint("loading")
        objects = rospy.get_param("fakehydro/objects")
        self.object_positions = {}
        self.topic_mapping = {}
        for obj in objects.keys():
            self.object_positions[obj] = None
            self.topic_mapping[objects[obj]["odom_topic"]] = obj
            rospy.Subscriber(objects[obj]["odom_topic"], Odometry, self.object_pose_callback)

        self.position_delta = rospy.get_param("fakehydro/position_delta")
        self.yaw_delta = (np.pi / 180.0) * rospy.get_param("fakehydro/yaw_delta_degrees")
        self.psi = (np.pi / 180.0) * rospy.get_param("fakehydro/psi_proximity_degrees")
        self.sigma = (np.pi / 180.0) * rospy.get_param("fakehydro/standard_deviation")
        meas_freq = rospy.get_param("fakehydro/meas_freq")
        self.active_obj = rospy.get_param("fakehydro/starting_object")
        self.seq = 0
        if self.active_obj == "random":
            self.active_obj = np.random.choice(self.object_positions.keys())
        self.print_pinger_status()

        self.active_pub = rospy.Publisher("hydrophones/active", String, queue_size=10)
        self.bias_pub = rospy.Publisher("hydrophones/debug_is_biased", Bool, queue_size=10)
        self.options_pub = rospy.Publisher("hydrophones/debug_pinger_options", PingerOptions, queue_size=10)
        self.debug_pub = rospy.Publisher("hydrophones/debug_measurements", PoseStamped, queue_size=10)
        self.pub = rospy.Publisher("hydrophones/measurements", Hydrophones, queue_size=10)
        rospy.Subscriber("/leviathan/description/pose_gt", Odometry, self.sub_pose_callback)

        self.biased = False
        self.bias_position = None
        self.bias_yaw = None
        self.bias = 0.0
        rospy.Timer(rospy.Duration(1 / meas_freq), self.publish_measurement)
        rospy.Service('hydrophones/switch_active_pinger', PingerSwitch, self.switch_active_pinger)

    def switch_active_pinger(self, req):
        active = req.active
        if active in self.object_positions.keys():
            self.active_obj = active
            self.print_pinger_status()
            return True
        else:
            self.cuprint(active + " is not a pinger choice", warn=True)
            return False

    def publish_measurement(self, msg):
        # self.cuprint("publishing")
        self.seq += 1
        obj_position = self.object_positions[self.active_obj]
        if obj_position == None:
            self.cuprint("haven't received " + self.active_obj + "'s pose_gt", warn=True)
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
            self.biased = False
        else:
            if not self.biased:
                self.bias = np.random.uniform(self.psi, - self.psi)
                self.bias_position = self.position
                self.bias_yaw = self.yaw
            else:
                x_diff = self.bias_position.x - self.position.x
                y_diff = self.bias_position.y - self.position.y
                z_diff = self.bias_position.z - self.position.z

                # Check if we've moved and need to recalculate the bias
                if np.linalg.norm([x_diff, y_diff, z_diff]) > self.position_delta:
                    self.bias = np.random.uniform(self.psi, - self.psi)
                    self.bias_position = self.position
                    self.bias_yaw = self.yaw
                elif abs(self.bias_yaw - self.yaw) > self.yaw_delta:
                    self.bias = np.random.uniform(self.psi, - self.psi)
                    self.bias_position = self.position
                    self.bias_yaw = self.yaw

                self.bias = np.random.uniform(self.psi, - self.psi)
            msg.azimuth = np.random.normal(self.bias, self.sigma)
        self.pub.publish(msg)
        b = Bool(); b.data = self.biased
        self.bias_pub.publish(b)
        s = String(); s.data = self.active_obj
        self.active_pub.publish(s)

        # Debug pub
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        quat = tf.transformations.quaternion_from_euler(0,0, msg.azimuth)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.debug_pub.publish(pose_msg)

        self.publish_pinger_options()

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
        if topic in self.topic_mapping.keys():
            self.object_positions[self.topic_mapping[topic]] = msg.pose.pose.position

    def sub_pose_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion([x,y,z,w])
        self.position = msg.pose.pose.position

    def publish_pinger_options(self):
        po = PingerOptions()
        po.options = self.object_positions.keys()
        self.options_pub.publish(po)

    def print_pinger_status(self):
        self.cuprint("active pinger set to " + bcolors.HEADER + self.active_obj + bcolors.ENDC)

if __name__ == "__main__":
    rospy.init_node("fakehydro")
    h = HydroFaker()
    rospy.spin()