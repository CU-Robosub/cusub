#!/usr/bin/env python

import rospy

import tf

import math

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

class ARTrack(object):

    def __init__(self):
        pass

    def handle_ar_pos(self, msg):

        for marker in msg.markers:

            # reject false hits
            if marker.id > 16:
                continue

            # Get the pose of the marker
            occam_pose = marker.pose
            occam_pose.header.frame_id = marker.header.frame_id
            occam_pose.header.stamp = marker.header.stamp

            # Get the distance to the tag to estimate covariance
            pos = occam_pose.pose.position
            dist = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)

            # Far things have a lot off error skip them
            if dist > 10.0:
                continue

            # Transform to baselink
            occam_pose = self.listener.transformPose("leviathan/description/base_link", occam_pose)

            # Invert the pose so we have it from the perspective of the marker
            trans = [occam_pose.pose.position.x, occam_pose.pose.position.y, occam_pose.pose.position.z]
            rot = [occam_pose.pose.orientation.x, occam_pose.pose.orientation.y, occam_pose.pose.orientation.z, occam_pose.pose.orientation.w]
            transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            inversed_transform = tf.transformations.inverse_matrix(transform)
            translation = tf.transformations.translation_from_matrix(inversed_transform)
            quaternion = tf.transformations.quaternion_from_matrix(inversed_transform)

            occam_pose.pose.position.x = translation[0]
            occam_pose.pose.position.y = translation[1]
            occam_pose.pose.position.z = translation[2]

            occam_pose.pose.orientation.x = quaternion[0]
            occam_pose.pose.orientation.y = quaternion[1]
            occam_pose.pose.orientation.z = quaternion[2]
            occam_pose.pose.orientation.w = quaternion[3]

            # Set the frame_id so the pose is from the perspective of the marker
            occam_pose.header.frame_id = "ar_tag_" + str(marker.id) + "_gt"

            # Transfrom the pose into worldspace
            world_baselink_pose = self.listener.transformPose("world", occam_pose)

            # Estimate covariance
            lin_cov = dist / 10.0

            # Feed pose to EKF
            world_baselink_pose_wc = PoseWithCovarianceStamped()
            world_baselink_pose_wc.pose.covariance = [lin_cov, 0, 0, 0, 0, 0,
                                                      0, lin_cov, 0, 0, 0, 0,
                                                      0, 0, lin_cov, 0, 0, 0,
                                                      0, 0, 0, 0.1, 0, 0,
                                                      0, 0, 0, 0, 0.1, 0,
                                                      0, 0, 0, 0, 0, 0.1]

            world_baselink_pose_wc.header = world_baselink_pose.header
            world_baselink_pose_wc.pose.pose = world_baselink_pose.pose

            self.ar_pub.publish(world_baselink_pose_wc)

    def depth_callback(self, data):
        self.depth_pub.publish(data)

    def dvl_callback(self, data):
        data.header.frame_id = "leviathan_ar/base_link"
        self.dvl_pub.publish(data)

    def imu_callback(self, data):
        data.header.frame_id = "leviathan_ar/base_link"
        self.imu_pub.publish(data)

    def run(self):

        rospy.init_node("occam_world_transform")

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.ar_pub = rospy.Publisher("cusub_common/ar/pose", PoseWithCovarianceStamped, queue_size=1)

        rospy.Subscriber("cusub_common/depth", PoseWithCovarianceStamped, self.depth_callback)
        rospy.Subscriber("cusub_common/dvl", TwistWithCovarianceStamped, self.dvl_callback)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)

        self.depth_pub = rospy.Publisher("cusub_common/ar/depth", PoseWithCovarianceStamped, queue_size=1)
        self.dvl_pub = rospy.Publisher("cusub_common/ar/dvl", TwistWithCovarianceStamped, queue_size=1)
        self.imu_pub = rospy.Publisher("cusub_common/ar/imu", Imu, queue_size=1)

        rospy.Subscriber("o0/ar_pose_marker", AlvarMarkers, self.handle_ar_pos)
        rospy.Subscriber("o1/ar_pose_marker", AlvarMarkers, self.handle_ar_pos)
        rospy.Subscriber("o2/ar_pose_marker", AlvarMarkers, self.handle_ar_pos)
        rospy.Subscriber("o3/ar_pose_marker", AlvarMarkers, self.handle_ar_pos)
        rospy.Subscriber("o4/ar_pose_marker", AlvarMarkers, self.handle_ar_pos)

        rospy.spin()

if __name__ == "__main__":
    ARTRACKER = ARTrack()
    ARTRACKER.run()
