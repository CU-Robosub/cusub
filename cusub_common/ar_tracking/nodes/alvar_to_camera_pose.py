#!/usr/bin/env python
"""
Converts alvar markers to camera poses
"""

import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers

from localizer.msg import CameraObjectPose

class AlvarToCameraPose(object):
    """Converts poses into sub pose estimates in the map frame
    for localization
    """

    def __init__(self):
        pass

    def ar_pose_callback(self, msg):
        """Takes in the pose and inverts it so we have the robot pose
        in the map frame for localization
        """

        for marker in msg.markers:

            # reject false hits
            if marker.id > 16:
                continue

            # Get the pose of the marker
            marker_pose = marker.pose
            marker_pose.header.frame_id = marker.header.frame_id
            marker_pose.header.stamp = marker.header.stamp

            # Send Camera Pose
            cam_obj_pose = CameraObjectPose()
            cam_obj_pose.object_pose = marker_pose
            cam_obj_pose.object_frame_id = rospy.get_namespace()[1:] \
                                           + "map/ar_tag_" + str(marker.id) + "_gt"

            self.map_pose_pub.publish(cam_obj_pose)

    def run(self):

        rospy.init_node("alvar_to_camera_pose")

        self.listener = tf.TransformListener()

        self.map_pose_pub = rospy.Publisher("cusub_common/cam_pose",
                                            CameraObjectPose, queue_size=1)

        # Listen to marker topics to generate camera object poses
        marker_topics = rospy.get_param("~marker_topics")
        for topic in marker_topics:
            rospy.Subscriber(topic, AlvarMarkers, self.ar_pose_callback)

        rospy.spin()

if __name__ == "__main__":
    ATCP = AlvarToCameraPose()
    ATCP.run()
