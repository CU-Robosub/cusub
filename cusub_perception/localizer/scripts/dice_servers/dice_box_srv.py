#!/usr/bin/env python
from __future__ import division
"""
Simple Classical CV Server for Dice
Estimates the position of dice by pnping the darknet corners of the boxes
"""
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from localizer.srv import ClassicalBoxes2Poses
from geometry_msgs.msg import Pose
import numpy as np
from pdb import set_trace
import cv2

class DiceBoxServer():


    # Dice box truth point indices in dice_truth_points
    # The center is illustrated by the 'X'
    # 1-------3
    # |       |
    # |   X   |
    # |       |
    # 0-------2

    # Each side is 22.86cm = .2286m
    # Maybe create a tuning function..?
    
    # Slightly inflated to account for darknet's natural resizing
    dice_truth_points = np.array([[-0.1143, -0.1143, 0],
                                  [-0.1143, 0.1143, 0],
                                  [0.1143, -0.1143, 0],
                                  [0.1143, 0.1143,  0]], dtype=np.float32)

    occam_camera_matrix = np.array([656.911402, 0.000000, 373.735385, 0.000000, 719.001469, 156.944858, 0.000000, 0.000000, 1.000000])

    occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000])

    
    def __init__(self):
        ns = rospy.get_namespace()
        self.server = rospy.Service(ns+"cusub_perception/localize_dice_box", ClassicalBoxes2Poses, self.localize)
        self.occam_camera_matrix.shape = (3,3)
        print(self.dice_truth_points)
        rospy.loginfo("Dice Box Initialized")
    def localize(self, req):
        poses = []
        classes = []
        for box in req.boxes:
            bot_left_pixel = (box.xmin, box.ymax)
            top_left_pixel = (box.xmin, box.ymin)
            bot_right_pixel = (box.xmax, box.ymax)
            top_right_pixel = (box.xmax, box.ymin)
            pt_arr = np.array([bot_left_pixel, top_left_pixel, bot_right_pixel, top_right_pixel], dtype=np.float32)
            print(pt_arr)
            # Call PnP
            retval, rvec, tvec = cv2.solvePnP(self.dice_truth_points, pt_arr, self.occam_camera_matrix, self.occam_distortion_coefs)

            # # PnP uses different coord system, do quick conversion
            pose = Pose()
            pose.position.x = tvec[2]
            pose.position.y = -1*tvec[0]
            pose.position.z = -1*tvec[1]
            poses.append(pose)
            print(pose)
            classes.append(box.Class)
            rospy.loginfo(box.Class + " : " + str(pose))
        return poses, classes
            
            

def main():
    rospy.init_node('dice_box')
    server = DiceBoxServer()
    rospy.spin()

if __name__ == "__main__":
    main()
