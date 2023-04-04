#!/usr/bin/env python

# sim subscriber - /leviathan/cusub_common/downcam/image_color
# sub subscriber - triton/down_cam/image_raw
# sub compressed - triton/down_cam/compressed

# ^^^ interpret with the python interpreter

import cv2 # image recognition class
import numpy as np # color array and finding mean values
import rospy # ros implementation
from cv_bridge import CvBridge # for converting ros images into openCV objects
from sensor_msgs.msg import Image
import os
import behavior as bt
from std_msgs.msg import Float64


class Nodo(object):
    def __init__(self):
        # parameters
        self.image = None
        self.br = CvBridge()
        # Node cycle rate in Hz
        self.loop_rate = rospy.Rate(1)
        # publishers
        self.pub = rospy.Publisher('/leviathan/cusub_common/motor_controllers/pid/drive/setpoint', Float64, queue_size=10)
        # Subscribers
        rospy.Subscriber("triton/down_cam/image_raw", Image, self.callback)
    
    def printDotArray(frame, contours):
        counter = 0
        cv2.rectangle(frame, (0, 0),(480, 480),(0,0,255),5)
        for i in range(8):
            for j in range(8):
                centerupper = ((480//8)*i + 30 + 10,(480//8)*j + 30 + 10)
                centerlower = ((480//8)*i + 30 - 10,(480//8)*j + 30 - 10)
                if(np.abs(cv2.pointPolygonTest(contours, centerupper, True)) < 30 and
                np.abs(cv2.pointPolygonTest(contours, centerlower, True)) < 30):
                    cv2.circle(frame, ((480//8)*i + 30,(480//8)*j + 30),10,(238, 245, 37),-1)
                    counter = counter + 1
                else:
                    cv2.circle(frame, ((480//8)*i + 30,(480//8)*j + 30),10,(0,0,255),-1)
        return counter
    
    def callback(self, msg):
        
        root = bt.Selector("root")
        
        condition = bt.Condition("move_condition", 'should_move') # set to true or false depending on where the red button is
        sequence = bt.Sequence("move", True)
        strafe_left = bt.Move("strafe_left", 'position')
        strafe_right = bt.Move("strafe_right", 'position')
        move_forward = bt.Move("move_forward", 'position')
        move_backward = bt.move("move_backward", 'position')
        alt_up = bt.Move("alt_up", 'position')
        alt_down = bt.Move("alt_down", 'position')
        
        # arrange nodes
        """
        root.nodes.append(condition)
        condition.nodes.append(sequence)
        sequence.nodes.append(strafe_left)
        sequence.nodes.append(strafe_right)
        sequence.nodes.append(move_forward)
        sequence.nodes.append(alt_up)
        sequence.nodes.append(alt_down)
        root.nodes.append(move_backward) # returns the sub to the starting position of the task, effectively making it exit the box
        
        # display the behavior tree
        bt.display_tree(root)
        """
        
        
        frame = self.br.imgmsg_to_cv2(msg,desired_encoding="bgr8") # converts image stream from ROS into openCV object. msg is ROS image stream
        height = frame.shape[0]
        width = frame.shape[1]

        l_b = np.array([10, 100, 20])  # lower hsv bound for red // 10, 100, 20
        u_b = np.array([25, 255, 255])  # upper hsv bound to red // 25, 255, 255
        cx, cy = 0, 0
        
        # ret, frame = video.read()
        print("callback being run")
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2BGR555)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, l_b, u_b)  # color range to look for

        _, contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # finds contours of object
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

            M = cv2.moments(contour)  # for finding the centroid of the rectangle
            if M["m00"] != 0:  # ensures no division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            
            # centroid dot
            center = cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            # centroid label
            ### Publish centroid to ros ###
            cv2.putText(rect, str(cx) + ", " + str(cy), (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
            
            cv2.imshow("feed", center)
        else:  # only runs when no contours (or objects to detect rather) are detected
            cv2.imshow("feed", frame)

        # array of dots for gripper cam resolution
        num_green = self.printDotArray(frame, approx)
        rospy.loginfo('Num green boxes recieved: ' + str(num_green))
        cv2.putText(frame, str(num_green), 
                    (width//3 - 20, height//3 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
        cv2.imshow("feed", frame)
        # show the mask feed
        cv2.imshow("mask", mask)

        #quit key
        c = cv2.waitKey(1)
        if c == 27:
            cv2.destroyAllWindows()
        # rospy.loginfo('Feed recieved')
        
    def execute(self):
        rospy.loginfo('Timing images...')
        while not rospy.is_shutdown(): # publish only while ros is active
            self.root.activate(bt.blackboard)
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()




if __name__ == '__main__':
    rospy.init_node("imagecapture", anonymous=True)
    my_node = Nodo()
    my_node.execute()
    
# https://www.youtube.com/watch?v=zPQdfm8VABA <-- adding models to gazebo for image testing