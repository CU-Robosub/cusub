#!/usr/bin/env python
"""
Estimates the pose of square of LEDs
"""

import rospy
import tf

import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from localizer.msg import CameraObjectPose

class MagicLEDPoseEstimator(object):
    """Estimates the pose of square of LEDs
    """

    def __init__(self):
        pass

    def image_callback(self, image):
        """Takes in camera image and estimates pose of any LED squares detected
        """

        try:
            img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img, self.lower_color_bounds, self.upper_color_bounds)

        #mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        #mask_img = img & mask_rgb
        
        keypoints = self.detector.detect(mask)

        if len(keypoints) == 4:

            points = []

            keypoints = keypoints[:4]

            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]

                points.append((x, y))

            # sort by x
            points.sort(key=lambda x:x[0])
            left_points = points[:2]
            right_points = points[2:]
            # sort by y
            left_points.sort(key=lambda y:y[1])
            right_points.sort(key=lambda y:y[1])

            # 1-------3
            # |       |
            # |   X   |
            # 0       2
            # truth points
            truth_points = np.array([[0, -0.625, -0.625],
                                     [0, -0.625,  0.625],
                                     [0,  0.625, -0.625],
                                     [0,  0.625,  0.625]], dtype=np.float32)

            pt_arr = np.array([left_points[1], left_points[0], right_points[1], right_points[0]], dtype=np.float32)
            pt_arr = np.ascontiguousarray(pt_arr[:,:2]).reshape((4,1,2))

            _, rvec, tvec = cv2.solvePnP(truth_points, pt_arr, self.camera_matrix, self.distortion_coefs, flags=cv2.SOLVEPNP_AP3P)

            print(pt_arr)

            pts, _ = cv2.projectPoints(truth_points, rvec, tvec, self.camera_matrix, self.distortion_coefs)
            for pt in pts:
                 cv2.circle(img,(int(pt[0][0]),int(pt[0][1])), 10, (0,255,0), -1)

            img = cv2.drawKeypoints(img, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("thresh", img)
            cv2.waitKey(1)

            print(tvec)

            self.br.sendTransform((tvec[0], tvec[1], tvec[2]),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                            "array",
                            "world")

    def run(self):
        """Node init and main loop"""

        rospy.init_node("magic_led_pose_estimator")

        self.br = tf.TransformBroadcaster()

        self.camera_matrix = np.array([744.514175, 0.000000, 292.868922, 0.000000, 759.244756, 309.052427, 0.000000, 0.000000, 1.000000])
        self.camera_matrix.shape = (3,3)
        self.distortion_coefs = np.array([-0.157287, 0.167617, 0.026414, 0.005206, 0.000000])

        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 255
        
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 1000

        params.filterByCircularity = False
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByColor = False
        params.blobColor = 255

        self.detector = cv2.SimpleBlobDetector_create(params)

        # The order of the colors is blue, green, red
        self.lower_color_bounds = np.array([220, 220, 220])
        self.upper_color_bounds = np.array([250, 250, 255])

        #self.listener = tf.TransformListener()

        self.bridge = CvBridge()

        rospy.Subscriber("camera_image", Image, self.image_callback)
        self.obj_pose_pub = rospy.Publisher("cam_pose",
                                            CameraObjectPose, queue_size=1)

        rospy.spin()

if __name__ == "__main__":
    MLPE = MagicLEDPoseEstimator()
    MLPE.run()
