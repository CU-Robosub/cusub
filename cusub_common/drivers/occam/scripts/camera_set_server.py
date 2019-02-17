#!/usr/bin/env python2
"""
Services for returing a false SetCameraInfo message
currently unable to write settings to camera
"""

import rospy
from sensor_msgs.srv import SetCameraInfo


def return_false(req):
    return [False, 'cannot set camera parameters']

def camera_set_server():
    rospy.init_node('occam_set_servers')
    s0 = rospy.Service('/occam/image0/set_camera_info', SetCameraInfo, return_false)
    s1 = rospy.Service('/occam/image1/set_camera_info', SetCameraInfo, return_false)
    s2 = rospy.Service('/occam/image2/set_camera_info', SetCameraInfo, return_false)
    s3 = rospy.Service('/occam/image3/set_camera_info', SetCameraInfo, return_false)
    s4 = rospy.Service('/occam/image4/set_camera_info', SetCameraInfo, return_false)

    print "services running"
    rospy.spin()

if __name__ == '__main__':
    camera_set_server()
