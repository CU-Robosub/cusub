#!/usr/bin/env python

# ^^^ interpret with the python interpreter

import cv2 # image recognition class
import numpy as np # color array and finding mean values
import rospy # ros implementation
from cv_bridge import CvBridge # for converting ros images into openCV objects
from sensor_msgs.msg import Image
import os


class Nodo(object):
    def __init__(self):
        # parameters
        self.image = None
        self.br = CvBridge()
        # Node cycle rate in Hz
        self.loop_rate = rospy.Rate(1)
        
        # publishers
        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/leviathan/cusub_common/downcam/image_color", Image, self.callback)
    
    def callback(self, msg):
        # 0 denotes capture from webcam - may need to change for robosub.
        frame = self.br.imgmsg_to_cv2(msg) # converts image stream from ROS into openCV object. msg is ROS image stream
        # video = cv2.VideoCapture()

        l_b = np.array([0, 200, 150])  # lower hsv bound for red
        u_b = np.array([275, 275, 250])  # upper hsv bound to red
        cx, cy = 0, 0
        
        # ret, frame = video.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(hsv, l_b, u_b)  # color range to look for

        _, contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # finds contours of object
        # print(str(contours) + str(_))
        if (contours):  # run only if there are contours found (prevents crashing)
            max_contour = contours[0]
            for contour in contours:
                if cv2.contourArea(contour) > cv2.contourArea(max_contour):
                    max_contour = contour
                contour = max_contour
                approx = cv2.approxPolyDP(
                    contour, 0.01*cv2.arcLength(contour, True), True)  # approximates the contour making it simpler for the box to be drawn around
                # set the x, y, width and height to bound approximations
                x, y, w, h = cv2.boundingRect(approx)

            # bounding box around object
            rect = cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 5)
            # centroid dot
            center = cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            # c centroid label
            ### Publish centroid to ros ###
            cv2.putText(rect, str(cx) + ", " + str(cy), (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

            M = cv2.moments(contour)  # for finding the centroid of the rectangle
            if M["m00"] != 0:  # ensures no division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            # finally show the feed
            # cv2.imshow("feed", rect)
            cv2.imshow("feed", center)
        else:  # only runs when no contours (or objects to detect rather) are detected
            cv2.imshow("feed", frame)

        # show the mask feed
        cv2.imshow("mask", mask)
        
        #quit key
        c = cv2.waitKey(1)
        if c == 27:
            cv2.destroyAllWindows()
        
    
        rospy.loginfo('Feed recieved')
        
    def execute(self):
        rospy.loginfo('Timing images...')
        while not rospy.is_shutdown(): # publish only while ros is active
            rospy.loginfo('publishing image...')
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()
                


# # 0 denotes capture from webcam - may need to change for robosub.
# video = cv2.VideoCapture(0)

# l_b = np.array([0, 200, 150])  # lower hsv bound for red
# u_b = np.array([275, 275, 250])  # upper hsv bound to red
# cx, cy = 0, 0
# while True:
#     ret, frame = video.read()

#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     # rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     mask = cv2.inRange(hsv, l_b, u_b)  # color range to look for

#     _, contours, _ = cv2.findContours(
#         mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # finds contours of object
#     # print(str(contours) + str(_))
#     if (contours):  # run only if there are contours found (prevents crashing)
#         max_contour = contours[0]
#         for contour in contours:
#             if cv2.contourArea(contour) > cv2.contourArea(max_contour):
#                 max_contour = contour
#             contour = max_contour
#             approx = cv2.approxPolyDP(
#                 contour, 0.01*cv2.arcLength(contour, True), True)  # approximates the contour making it simpler for the box to be drawn around
#             # set the x, y, width and height to bound approximations
#             x, y, w, h = cv2.boundingRect(approx)

#         # bounding box around object
#         rect = cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 5)
#         # centroid dot
#         center = cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
#         # c centroid label
#         ### Publish centroid to ros ###
#         cv2.putText(rect, str(cx) + ", " + str(cy), (x, y-10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

#         M = cv2.moments(contour)  # for finding the centroid of the rectangle
#         if M["m00"] != 0:  # ensures no division by zero
#             cx = int(M["m10"] / M["m00"])
#             cy = int(M["m01"] / M["m00"])
#         else:
#             cx, cy = 0, 0
#         # finally show the feed
#         # cv2.imshow("feed", rect)
#         cv2.imshow("feed", center)
#     else:  # only runs when no contours (or objects to detect rather) are detected
#         cv2.imshow("feed", frame)

#     # show the mask feed
#     cv2.imshow("mask", mask)

#     # quit key
#     c = cv2.waitKey(1)
#     if c == 27:
#         break
# cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("imagecapture", anonymous=True)
    my_node = Nodo()
    my_node.execute()