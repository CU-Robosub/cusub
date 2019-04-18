#!/usr/bin/env python
"""
Module facilitates pose estimation in map frame for localization
"""

import math

import rospy
import tf

import tf.transformations as transform

from geometry_msgs.msg import PoseWithCovarianceStamped

from localizer.msg import CameraObjectPose

class CameraPoseToMap(object):
    """Converts poses into sub pose estimates in the map frame
    for localization
    """

    """Transform listener
    """
    listener = None
    """Publishes map poses
    """
    map_pose_pub = None

    def __init__(self):
        pass

    def pose_callback(self, cam_obj_pose):
        """Takes in the pose and inverts it so we have the robot pose
        in the map frame for localization
        """

        cam_pose = cam_obj_pose.object_pose

        # Get the distance to the tag to estimate covariance
        pos = cam_pose.pose.position
        dist = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)

        # Far things have a lot of error skip them
        if dist > 15.0:
            return

        # Transform to baselink
        cam_pose = self.listener.transformPose(rospy.get_namespace()[1:]
                                               + "description/base_link", cam_pose)

        # Invert the pose so we have it from the perspective of the marker
        trans = [cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z]
        rot = [cam_pose.pose.orientation.x, cam_pose.pose.orientation.y,
               cam_pose.pose.orientation.z, cam_pose.pose.orientation.w]
        trans = transform.concatenate_matrices(transform.translation_matrix(trans),
                                               transform.quaternion_matrix(rot))
        inversed_transform = transform.inverse_matrix(trans)
        translation = transform.translation_from_matrix(inversed_transform)
        quaternion = transform.quaternion_from_matrix(inversed_transform)

        cam_pose.pose.position.x = translation[0]
        cam_pose.pose.position.y = translation[1]
        cam_pose.pose.position.z = translation[2]

        cam_pose.pose.orientation.x = quaternion[0]
        cam_pose.pose.orientation.y = quaternion[1]
        cam_pose.pose.orientation.z = quaternion[2]
        cam_pose.pose.orientation.w = quaternion[3]

        # Set the frame_id so the pose is from the perspective of the object
        cam_pose.header.frame_id = cam_obj_pose.object_frame_id

        # Transfrom the pose into worldspace
        # [1:] removes leading /
        world_baselink_pose = self.listener.transformPose(rospy.get_namespace()[1:]
                                                          + "description/map", cam_pose)

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
        """ Initalizes node and spins
        """

        rospy.init_node("camera_pose_to_map")

        self.listener = tf.TransformListener()

        self.map_pose_pub = rospy.Publisher("cusub_common/map_pose",
                                            PoseWithCovarianceStamped, queue_size=1)

        rospy.Subscriber("cusub_common/cam_pose", CameraObjectPose, self.pose_callback)

        rospy.spin()

if __name__ == "__main__":
    CPTM = CameraPoseToMap()
    CPTM.run()
