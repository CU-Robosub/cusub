#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np

class camera_multiplexer(object):

    def __init__(self):
        self.yolo_pipe = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)

    def wait_pub_rotate(self, delay):
        rate = rospy.Rate(delay)
        while not rospy.is_shutdown():
        
            image0 = rospy.wait_for_message('/occam/image0', Image)
            image0.header.frame_id = "occam0_frame"
            self.yolo_pipe.publish(image0)
            rospy.loginfo("sent image0")
            rate.sleep()

           # image1 = rospy.wait_for_message('/occam/image1', Image)
           # image1.header.frame_id = "occam1_frame"
           # self.yolo_pipe.publish(image1)
           # rospy.loginfo("sent image1")
           # rate.sleep()

           # image4 = rospy.wait_for_message('/occam/image4', Image)
           # image4.header.frame_id = "occam4_frame"
           # self.yolo_pipe.publish(image4)
           # rospy.loginfo("sent image4")
           # rate.sleep()


if __name__ == '__main__':

    rospy.init_node('occam2cam')
    multiplexer = camera_multiplexer()
    multiplexer.wait_pub_rotate(1)
