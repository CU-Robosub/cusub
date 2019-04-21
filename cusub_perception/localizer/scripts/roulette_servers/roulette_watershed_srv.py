#!/usr/bin/env python
from __future__ import division

import rospy

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from localizer.srv import ClassicalBoxes2Poses

from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError

import tf

import numpy as np
import cv2

class RouletteWatershedServer():

    METHOD_MAXMIN = 1
    METHOD_CONTOUR = 2

    method = METHOD_CONTOUR

    debug = False

    # 2-3
    # | |
    # 1 4
    roulette_truth_points = np.array([[-0.22, -0.36, 0],
                                      [-0.22,  0.36, 0],
                                      [ 0.22,  0.36, 0],
                                      [ 0.22, -0.36,  0]], dtype=np.float32)

    downcam_camera_matrix = np.array([344.133456, 0.000000, 663.520917, 0.000000, 344.629709, 478.643504, 0.000000, 0.000000, 1.000000])
    downcam_distortion_coefs = np.array([0.035690, -0.010044, 0.000537, -0.000878, 0.000000])

    
    def __init__(self):

        ns = rospy.get_namespace()

        self.server = rospy.Service(ns+"cusub_perception/localize_roulette_watershed", ClassicalBoxes2Poses, self.localize)
        self.downcam_camera_matrix.shape = (3,3)

        # print(self.roulette_truth_points)

        rospy.loginfo("Roulette Watershed Initialized")

        self.bridge = CvBridge()

    def order_points(self, points):

        l1 = np.linalg.norm(np.array(points[0]) - np.array(points[1]))
        l2 = np.linalg.norm(np.array(points[2]) - np.array(points[1]))

        pr_arr = None
        if(l2 > l1):
            # top to right is long side
            pt_arr = np.array([points[3], points[0], points[1], points[2]], dtype=np.float32)
        else:
            # top to left is long side
            pt_arr = np.array([points[0], points[1], points[2], points[3]], dtype=np.float32)

        return pt_arr

    def points_by_max_minima(self, image, markers, debug_img):
        """
        This function returns the points for the roulette outline
        by looking at left right top bottom maximums
        """

        marker_pos = np.argwhere(markers == 2)

        marker_xmin_idx = np.argmin(marker_pos[:,0])
        marker_xmin_x = marker_pos[marker_xmin_idx,0]
        marker_xmin_y = marker_pos[marker_xmin_idx,1]
        marker_left = (marker_xmin_y, marker_xmin_x)

        marker_xmax_idx = np.argmax(marker_pos[:,0])
        marker_xmax_x = marker_pos[marker_xmax_idx,0]
        marker_xmax_y = marker_pos[marker_xmax_idx,1]
        marker_right = (marker_xmax_y, marker_xmax_x)

        marker_ymin_idx = np.argmin(marker_pos[:,1])
        marker_ymin_x = marker_pos[marker_ymin_idx,0]
        marker_ymin_y = marker_pos[marker_ymin_idx,1]
        marker_top = (marker_ymin_y, marker_ymin_x)

        marker_ymax_idx = np.argmax(marker_pos[:,1])
        marker_ymax_x = marker_pos[marker_ymax_idx,0]
        marker_ymax_y = marker_pos[marker_ymax_idx,1]
        marker_bottom = (marker_ymax_y, marker_ymax_x)

        pt_arr = self.order_points([marker_left, marker_top, marker_right, marker_bottom])

        if debug_img is not None:
            pt_arr_2 = tuple(map(tuple, pt_arr))
            cv2.circle(debug_img, pt_arr_2[0], 5, (0,0,255), -1)
            cv2.circle(debug_img, pt_arr_2[1], 5, (0,0,255), -1)
            cv2.circle(debug_img, pt_arr_2[2], 5, (0,0,255), -1)
            cv2.circle(debug_img, pt_arr_2[3], 5, (0,0,255), -1)

        return pt_arr, False

    def get_points_by_contour(self, image, markers, debug_img):
        """
        This functino returns points of the roulette by
        taking the contour and finding the polygon approximation
        """

        ret, thresh = cv2.threshold(np.uint8(markers), 127, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        epsilon = 0.1*cv2.arcLength(contours[2],True)
        approx = cv2.approxPolyDP(contours[2], epsilon, True)

        if len(approx) != 4:
            # Fail
            return None, None

        pt_arr = self.order_points(approx)

        # divide polygon in 2 longitudinaly
        # find midpoints 2-3 1-4
        midpt_23 = (pt_arr[1] + pt_arr[2]) / 2.0
        midpt_14 = (pt_arr[0] + pt_arr[3]) / 2.0

        contour_top = np.array([pt_arr[1], midpt_23, midpt_14, pt_arr[0]]).reshape((-1,1,2)).astype(np.int32)
        contour_bot = np.array([pt_arr[2], midpt_23, midpt_14, pt_arr[3]]).reshape((-1,1,2)).astype(np.int32)

        # create mask of 1 half of roulette
        shape = image.shape
        shape = (shape[0], shape[1], 1)

        mask = np.zeros(shape, np.uint8)
        cv2.drawContours(mask, [contour_bot], -1, (255), -1)

        green_low = 40
        green_high = 75

        red_low = 170
        red_high = 10

        NOT_SURE = 0
        IS_RED = 1
        IS_GREEN = 2

        color_bot = NOT_SURE
        color_top = NOT_SURE

        # find color
        color = cv2.mean(image, mask=mask)
        color = np.array([[[color[0], color[1], color[2]]]], dtype=np.uint8)
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        hue = hsv[0][0][0]
        print('bot_hue', hue)

        if hue > green_low and hue < green_high:
            color_bot = IS_GREEN
        elif hue > red_low or hue < red_high:
            color_bot = IS_RED

        mask = np.zeros(shape, np.uint8)
        cv2.drawContours(mask, [contour_top], -1, (255), -1)

        color = cv2.mean(image, mask=mask)
        color = np.array([[[color[0], color[1], color[2]]]], dtype=np.uint8)
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        hue = hsv[0][0][0]
        print('top_hue', hue)

        if hue > green_low and hue < green_high:
            color_top = IS_GREEN
        elif hue > red_low or hue < red_high:
            color_top = IS_RED

        if color_top == IS_GREEN and color_bot == IS_RED:
            do_rotate = False
        elif color_top == IS_RED and color_bot == IS_GREEN:
            do_rotate = True
        else:
            # We dont know color!
            return None

        if debug_img is not None:

            # Show contours
            #cv2.drawContours(debug_img, contours, -1, (0,255,0), 3)

            # show approx polygon
            cv2.polylines(debug_img,[approx],True,(0,255,255))

            # show midpoints
            midpt_14_t = map(tuple, midpt_14)[0]
            midpt_23_t = map(tuple, midpt_23)[0]
            cv2.circle(debug_img, midpt_14_t, 5, (0,0,255), -1)
            cv2.circle(debug_img, midpt_23_t, 5, (0,0,255), -1)

        return pt_arr, do_rotate

    def localize(self, req):

        img = self.bridge.imgmsg_to_cv2(req.image, "passthrough")

        debug_img = None
        if self.debug:
            debug_img = img.copy()

        # Fill Background
        markers = np.ones((req.image.height, req.image.width), dtype=np.int32)

        marker_count = 2
        poses = []
        classes = []
        for box in req.boxes:

            if box.Class != "roulette":
                continue

            box_margin = 20

            # Clear Bounding Box
            markers[box.ymin-box_margin:box.ymax+box_margin, box.xmin-box_margin:box.xmax+box_margin] = 0
            #cv2.rectangle(debug_img, (box.xmin-box_margin, box.ymin-box_margin), (box.xmax+box_margin, box.ymax+box_margin), (0,255,0), 3)

            # Add marker
            box_width = box.xmax - box.xmin
            box_height = box.ymax - box.ymin
            box_x_center = int(box.xmin + box_width / 2.0)
            box_y_center = int(box.ymin + box_height / 2.0)
            marker_width = 0.25
            m_xmin = int(box_x_center - box_width * marker_width / 2.0)
            m_xmax = int(box_x_center + box_width * marker_width / 2.0)
            m_ymin = int(box_y_center - box_height * marker_width / 2.0)
            m_ymax = int(box_y_center + box_height * marker_width / 2.0)
            markers[m_ymin:m_ymax, m_xmin:m_xmax] = marker_count
            #cv2.rectangle(debug_img, (m_xmin, m_ymin), (m_xmax, m_ymax), (0,0,255), 3)

            marker_count += 1

        if marker_count == 2: # nothing found
            return None, None

        cv2.watershed(img, markers)

        if self.method == self.METHOD_MAXMIN:
            pt_arr, do_rotate = self.points_by_max_minima(img, markers, debug_img)
        elif self.method == self.METHOD_CONTOUR:
            pt_arr, do_rotate = self.get_points_by_contour(img, markers, debug_img)
        else:
            # Fail
            return None, None

        if pt_arr is None:
            # Fail
            return None, None

        _, rvec, tvec = cv2.solvePnP(self.roulette_truth_points, pt_arr, self.downcam_camera_matrix, self.downcam_distortion_coefs)

        # Convert to 3x3 Rotation Matrix
        rmat, _ = cv2.Rodrigues(rvec)

        # Convert to 4x4 Rotation Matrix
        rmat = np.hstack((rmat, np.array([[0],[0],[0]])))
        rmat = np.vstack((rmat, np.array([0,0,0,1])))

        # Convert to Quaternion
        rquat = tf.transformations.quaternion_from_matrix(rmat)

        # Flip rotation

        if do_rotate:

            # Quat to axis angle
            axis = np.array([0.0, 0.0, 0.0])
            angle = 2 * np.arccos(rquat[3])
            if (1 - (rquat[3] * rquat[3]) < 0.000001):
                axis[0] = rquat[0]
                axis[1] = rquat[1]
                axis[2] = rquat[2]
            else:
                s = np.sqrt(1 - (rquat[3] * rquat[3]))
                axis[0] = rquat[0] / s
                axis[1] = rquat[1] / s
                axis[2] = rquat[2] / s

            # rotate angle 180
            angle += 3.1415

            # axis to quat
            half_angle = angle / 2.0
            s = np.sin(half_angle)

            rquat[0] = axis[0] * s
            rquat[1] = axis[1] * s
            rquat[2] = axis[2] * s
            rquat[3] = np.cos(half_angle)

        pose = Pose()

        pose.position.x = tvec[0]   
        pose.position.y = tvec[1]
        pose.position.z = tvec[2]

        pose.orientation.x = rquat[0]
        pose.orientation.y = rquat[1]
        pose.orientation.z = rquat[2]
        pose.orientation.w = rquat[3]

        poses.append(pose)

        if self.debug:
          print(pose)

        box = req.boxes[0]
        classes.append(box.Class)
        rospy.loginfo(box.Class + " : " + str(pose))

        """
        res = cv2.convertScaleAbs(markers)
        _, res = cv2.threshold(res,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.imshow('image', res)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """

        # cv2.imshow('image', markers)

        if self.debug:
            cv2.imshow('Roulette Watershed', debug_img)
            cv2.waitKey(1)

        return poses, classes

def main():
    rospy.init_node('roulette_watershed')
    server = RouletteWatershedServer()
    rospy.spin()

class PsudoRequest():
    image = None
    boxes = []
    def __init__(self):
        pass

def test1():
    server = RouletteWatershedServer()
    server.debug = True

    bridge = CvBridge()
    cv_image = cv2.imread('test.jpg', 3)
    image_message = bridge.cv2_to_imgmsg(cv_image, "passthrough")

    roulette_box = BoundingBox()
    roulette_box.Class = "roulette"
    roulette_box.xmin = 793
    roulette_box.ymin = 415
    roulette_box.xmax = 1000
    roulette_box.ymax = 600
    
    req = PsudoRequest()
    req.image = image_message
    req.boxes.append(roulette_box)

    server.localize(req)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # test1()
    main()
