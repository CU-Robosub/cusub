#!/usr/bin/env python
import numpy as np
import argparse
import cv2
import ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MusscleTask:
    def __init__(self,topic):
        self.bridge = CvBridge()

        rospy.Subscriber(topic,Image,self.image_callback)

        self.blob_pub = rospy.Publisher("/mussle_blob",Image,queue_size=10)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def image_callback(self,msg):
        # print("hello")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image = np.array(frame,dtype=np.uint8)
        np.save("mussle.npy",self.image)
        self.blob_pub.publish(self.blob(self.image))
        

    def blob(self,image):
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(gray)
        # print(keypoints)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return self.bridge.cv2_to_imgmsg(im_with_keypoints)
        # Show keypoints
        cv2.imshow("Blob", im_with_keypoints)
        cv2.waitKey(0)


if __name__ == "__main__":
    rospy.init_node("musscle")
    mt = MusscleTask("/leviathan/description/camera_8/image_raw")