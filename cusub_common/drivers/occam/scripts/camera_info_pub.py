#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def create_cam_info_message(camera_info_namespace):
    camera_info = CameraInfo()
    camera_info.height = camera_info_namespace["height"]
    camera_info.width = camera_info_namespace["width"]
    camera_info.distortion_model = camera_info_namespace["distortion_model"]
    camera_info.D = camera_info_namespace["D"]
    camera_info.K = camera_info_namespace["K"]
    camera_info.R = camera_info_namespace["R"]
    camera_info.P = camera_info_namespace["P"]
    return camera_info

if __name__ == '__main__':
    rospy.init_node("occam_camera_info_pub", anonymous=True)    
    camera_info_namespace = rospy.get_param("~camera_info_namespace")
    cam_info_msg = create_cam_info_message(rospy.get_param(camera_info_namespace))
    cam_info_msg.header.frame_id = rospy.get_param("~camera_frame")
    pub_rate = rospy.get_param("~pub_rate")

    camera_info_topic = rospy.get_param("~camera_info_topic")
    pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=10)
    
    r = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        cam_info_msg.header.stamp = rospy.get_rostime()
        cam_info_msg.header.seq += 1
        pub.publish(cam_info_msg)
        r.sleep()