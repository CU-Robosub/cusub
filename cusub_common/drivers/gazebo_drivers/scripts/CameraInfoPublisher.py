#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher():

    def __init__(self):
        pass

    def run(self):

        self.seq = 0

        cameraInfoTopic = rospy.get_param('~camera_info_topic')
        frameID = rospy.get_param('~frameid')

        cameraInfoPublisher = rospy.Publisher(cameraInfoTopic, CameraInfo, queue_size=1)

        cameraInfo = CameraInfo()
        cameraInfo.header.seq = self.seq
        self.seq += 1
        cameraInfo.header.stamp = rospy.get_rostime()
        cameraInfo.header.frame_id = frameID
        cameraInfo.height = rospy.get_param('~height')
        cameraInfo.width = rospy.get_param('~width')
        cameraInfo.distortion_model = "plumb_bob"
        cameraInfo.D = rospy.get_param('~D')
        cameraInfo.K = rospy.get_param('~K')
        cameraInfo.R = rospy.get_param('~R')
        cameraInfo.P = rospy.get_param('~P')
        cameraInfo.binning_x = 0
        cameraInfo.binning_y = 0
        cameraInfo.roi.x_offset = 0
        cameraInfo.roi.y_offset = 0
        cameraInfo.roi.height = 0
        cameraInfo.roi.width = 0
        cameraInfo.roi.do_rectify = False

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            cameraInfo.header.seq = self.seq
            self.seq += 1

            cameraInfoPublisher.publish(cameraInfo)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('CameraInfoPublisher', anonymous=True)
    a = CameraInfoPublisher()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("CameraInfoPublisher has died!");