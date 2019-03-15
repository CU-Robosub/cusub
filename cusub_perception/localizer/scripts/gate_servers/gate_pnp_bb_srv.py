#!/usr/bin/env python
"""
Classical CV Server for StartGate
Localizes the gate via a PNP transform on the tops and bottoms of the gate
- Assumes our camera is horizontal
- Generates a pose by relating the image points to what we know of the true structure in 3D space (pnp transformation)
"""

import numpy as np
import cv2
import rospy
import tf
from localizer.srv import ClassicalBoxes2Poses
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

class GatePNPBoundingBoxServer():

    # pole_points indices on the gate are ordered as follows:
    # The center is illustrated by the 'X'
    # 1-------3
    # |       |
    # |   X   |
    # 0       2
    # Take note the indices are given by where the orange tape on the poles ends, not the bottom of the poles
    pole_truth_points = np.array([[0, -1.6, -0.6],
                                  [0, -1.6,  0.6],
                                  [0,  1.6, -0.6],
                                  [0,  1.6,  0.6]], dtype=np.float32)

    occam_camera_matrix = np.array([656.911402, 0.000000, 373.735385, 0.000000, 719.001469, 156.944858, 0.000000, 0.000000, 1.000000])

    occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000])

    def __init__(self):

        self.server = rospy.Service("cusub_perception/localize_gate_pnp_bb", ClassicalBoxes2Poses, self.localize)
        self.occam_camera_matrix.shape = (3,3)

        rospy.loginfo("Gate PNP Bounding Box Initialized")

    def localize(self, req):

        if len(req.boxes) < 2: # We need 2 bounding boxes
            return [Pose()], ['Failed']

        box1 = req.boxes[0]
        box2 = req.boxes[1]
        if box1.xmax < box2.xmax: # box1 is left
            left = box1
            right = box2
        else:
            left = box2
            right = box1

        top_pixel_l = ((left.xmin + left.xmax) / 2.0, left.ymax)
        bot_pixel_l = ((left.xmin + left.xmax) / 2.0, left.ymin)
        top_pixel_r = ((right.xmin + right.xmax) / 2.0, right.ymax)
        bot_pixel_r = ((right.xmin + right.xmax) / 2.0, right.ymin)

        pt_arr = np.array([bot_pixel_l, top_pixel_l, bot_pixel_r, top_pixel_r], dtype=np.float32)
        pt_arr = np.ascontiguousarray(pt_arr[:,:2]).reshape((4,1,2))

        _, rvec, tvec = cv2.solvePnP(self.pole_truth_points, pt_arr, self.occam_camera_matrix, self.occam_distortion_coefs, flags=cv2.SOLVEPNP_ITERATIVE)

        # Convert to 3x3 Rotation Matrix
        rmat, _ = cv2.Rodrigues(rvec)

        # Convert to 4x4 Rotation Matrix
        rmat = np.hstack((rmat, np.array([[0],[0],[0]])))
        rmat = np.vstack((rmat, np.array([0,0,0,1])))

        # Convert to Quaternion
        rquat = tf.transformations.quaternion_from_matrix(rmat)

        pose = Pose()

        pose.position.x = tvec[0]
        pose.position.y = tvec[1]
        pose.position.z = tvec[2]

        pose.orientation.x = rquat[0]
        pose.orientation.y = rquat[1]
        pose.orientation.z = rquat[2]
        pose.orientation.w = rquat[3]

        return [pose], ['start_gate']

if __name__ == "__main__":
    rospy.init_node("gate_pnp_bounding_box")
    server = GatePNPBoundingBoxServer()
    rospy.spin()
