#!/usr/bin/env python

"""
Thanks to Github user @rossbar for writing this script
https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo():
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "leviathan/description/stereo_down_right_cam_frame_optical"
    camera_info_msg.width = rospy.get_param("~image_width")
    camera_info_msg.height = rospy.get_param("~image_height")
    camera_info_msg.K = rospy.get_param("~camera_matrix/data")
    camera_info_msg.D = rospy.get_param("~distortion_coefficients/data")
    camera_info_msg.R = rospy.get_param("~rectification_matrix/data")
    camera_info_msg.P = rospy.get_param("~projection_matrix/data")
    camera_info_msg.distortion_model = rospy.get_param("~distortion_model")
    return camera_info_msg

if __name__ == "__main__":


    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo()

    publisher = rospy.Publisher("camera_info_2", CameraInfo, queue_size=10)
    rate = rospy.Rate(100)

    # Run publisher
    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        publisher.publish(camera_info_msg)
        rate.sleep()