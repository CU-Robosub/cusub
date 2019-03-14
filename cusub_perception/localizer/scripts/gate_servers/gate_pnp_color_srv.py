#!/usr/bin/env python
from __future__ import division
"""
Classical CV Server for StarGate
Localizes the gate via a PNP transform on the tops and bottoms of the gate
- Uses color characteristics to find the tops and bottoms of the poles
- Generates a pose by relating the image points to what we know of the true structure in 3D space (pnp transformation)
"""

import numpy as np
import cv2
import rospy
import tf
from localizer.srv import ClassicalBoxes2Poses
from geometry_msgs.msg import Pose
from classical_cv.gate_pnp_color import GatePNPAnalyzer
from cv_bridge import CvBridge, CvBridgeError

class ExpWeightedAvg():

    num_poses = 0
    def __init__(self, num_poses_to_avg):
        self.beta = (1 - num_poses_to_avg) / ( - num_poses_to_avg)
#        print "Beta: {}".format(self.beta)
        self.avg_pose = Pose()
#        print self.avg_pose

    def get_new_avg_pose(self, new_pose):
        # print "New pose: {}".format(new_pose)
        # print "Avg pose: {}".format(self.avg_pose)
        # print "Iter: {}".format(self.num_poses)
        self.num_poses += 1
        if self.num_poses == 1:
            self.avg_pose = new_pose

        x_avg = self.avg_pose.position.x
        y_avg = self.avg_pose.position.y
        z_avg = self.avg_pose.position.z

        new_x_avg = ( self.beta * x_avg ) + (1-self.beta) * (new_pose.position.x)
        new_y_avg = ( self.beta * y_avg ) + (1-self.beta) * (new_pose.position.y)
        new_z_avg = ( self.beta * z_avg ) + (1-self.beta) * (new_pose.position.z)

#        print self.beta ** self.num_poses
        # new_x_avg_corrected = new_x_avg / (1 - self.beta ** self.num_poses )
        # new_y_avg_corrected = new_y_avg / (1 - self.beta ** self.num_poses )
        # new_z_avg_corrected = new_z_avg / (1 - self.beta ** self.num_poses )

        pose = Pose()
        pose.position.x = new_x_avg
        pose.position.y = new_y_avg
        pose.position.z = new_z_avg
        self.avg_pose = pose

        return self.avg_pose


class GatePNPColorServer():

    # pole_points indices on the gate are ordered as follows:
    # The center is illustrated by the 'X'
    # 1-------3
    # |       |
    # |   X   |
    # 0       2
    # Take note the indices are given by where the orange tape on the poles ends, not the bottom of the poles
    pole_truth_points = np.array([[-1.6, -0.6, 0],
                                  [-1.6, 0.6, 0],
                                  [1.6, -0.6, 0],
                                  [1.6, 0.6,  0]], dtype=np.float32)

    occam_camera_matrix = np.array([656.911402, 0.000000, 373.735385, 0.000000, 719.001469, 156.944858, 0.000000, 0.000000, 1.000000])

    occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000])

    def __init__(self):
        ns = rospy.get_namespace()
        self.server = rospy.Service(ns +"localize_gate_pnp_color", ClassicalBoxes2Poses, self.localize)
        self.occam_camera_matrix.shape = (3,3)
        rospy.loginfo("Gate PNP Color Initialized")
        self.expAvg = ExpWeightedAvg( 5 ) # average over five poses

    def localize(self, req):

        if len(req.boxes) < 2: # We need 2 bounding boxes
            return [Pose()], ['Failed']

        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(req.image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
            return

        box1 = req.boxes[0]
        box2 = req.boxes[1]
        if box1.xmax < box2.xmax: # box1 is left
            left = box1
            right = box2
        else:
            left = box2
            right = box1

        gate_analyzer = GatePNPAnalyzer(img, "bgr")

        top_pixel_l, bot_pixel_l = gate_analyzer.localize_leg(left.xmin, \
                                                              left.ymin, \
                                                              left.xmax, \
                                                              left.ymax)
        top_pixel_r, bot_pixel_r = gate_analyzer.localize_leg(right.xmin, \
                                                              right.ymin, \
                                                              right.xmax, \
                                                              right.ymax)

        pt_arr = np.array([bot_pixel_l, top_pixel_l, bot_pixel_r, top_pixel_r], dtype=np.float32)

        #pt_arr = np.ascontiguousarray(pt_arr[:,:2]).reshape((4,1,2))

        _, rvec, tvec = cv2.solvePnP(self.pole_truth_points, pt_arr, self.occam_camera_matrix, self.occam_distortion_coefs, flags=cv2.SOLVEPNP_ITERATIVE)

        print(rvec)

        """
        points = cv2.projectPoints(self.pole_truth_points, rvec, tvec, self.occam_camera_matrix, self.occam_distortion_coefs)
        for pt in pt_arr:
            cv2.circle(img, (pt[0], pt[1]), 3, (255,0,0), 2)
        for pt in points[0]:
            cv2.circle(img, (pt[0][0], pt[0][1]), 3, (0,0,255), 2)
        cv2.imshow("debug", img)
        cv2.waitKey(0)
        """

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

        # TODO move this to the mapper
        # pose = self.expAvg.get_new_avg_pose(pose)

        return [pose], ['start_gate']

if __name__ == "__main__":
    rospy.init_node("gate_pnp_color")
    server = GatePNPColorServer()
    rospy.spin()
