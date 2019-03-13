#!/usr/bin/env python
"""
Module facilitates pose estimation in map frame for localization
"""

import math

import rospy
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped

from ar_track_alvar_msgs.msg import AlvarMarkers

class PoseToMap(object):
    """Converts poses into sub pose estimates in the map frame
    for localization
    """

    def __init__(self):
        pass

    def pose_callback(self, msg):
        """Takes in the pose and inverts it so we have the robot pose
        in the map frame for localization
        """

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
            # [1:] removes leading /
            world_baselink_pose = self.listener.transformPose(rospy.get_namespace()[1:] + "description/map", occam_pose)

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

            self.map_pose_pub.publish(world_baselink_pose_wc)

    def run(self):

        rospy.init_node("occam_world_transform")

        self.listener = tf.TransformListener()

        self.map_pose_pub = rospy.Publisher("cusub_common/map_pose", PoseWithCovarianceStamped, queue_size=1)

        rospy.Subscriber("o0/ar_pose_marker", AlvarMarkers, self.pose_callback)
        rospy.Subscriber("o1/ar_pose_marker", AlvarMarkers, self.pose_callback)
        rospy.Subscriber("o2/ar_pose_marker", AlvarMarkers, self.pose_callback)
        rospy.Subscriber("o3/ar_pose_marker", AlvarMarkers, self.pose_callback)
        rospy.Subscriber("o4/ar_pose_marker", AlvarMarkers, self.pose_callback)

        rospy.spin()

if __name__ == "__main__":
    PTM = PoseToMap()
    PTM.run()
