#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv

from cv_bridge import CvBridge

from cusub_print.cuprint import CUPrint

MIN_MATCH_COUNT = 10

class HomographySiftMatcher():

    def __init__(self):

        self.bridge = CvBridge()

        # Initiate SIFT detector
        self.sift = cv.xfeatures2d.SIFT_create()

        #img1_fn = '/opt/vampire_cute.png'
        img1_fn = '/opt/dracula.png'
        self.img1 = cv.imread(img1_fn,0) # queryImage
        self.kp1, self.des1 = self.sift.detectAndCompute(self.img1,None)

        self.occam_camera_matrix = np.array([656.926137, 0.000000, 376.5, 0.000000, 656.926137, 240.5, 0.000000, 0.000000, 1.000000])
        self.occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000])
        self.occam_camera_matrix.shape = (3,3)

        self.debug_pub = rospy.Publisher('homography_debug', Image, queue_size=1)

        rospy.Subscriber('/leviathan/cusub_common/occam/image0', Image, self.match_image_callback, queue_size=1, buff_size=2**24)

    def match_image_callback(self, image):

        #try:
        if True:

            img2 = self.bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')

            # find the keypoints and descriptors with SIFT
            kp2, des2 = self.sift.detectAndCompute(img2,None)
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(self.des1,des2,k=2)

            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

            if len(good) > MIN_MATCH_COUNT:

                src_pts = np.float32([ self.kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()

                h,w = self.img1.shape

                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts,M)
                img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)

                draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                                singlePointColor = None,
                                matchesMask = matchesMask, # draw only inliers
                                flags = 2)

                img3 = cv.drawMatches(self.img1,self.kp1,img2,kp2,good,None,**draw_params)

                # Draw target
                pts = np.float32([[260, 80]]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts,M)
                img3 = cv.circle(img3, (int(dst[0][0][0]+w), dst[0][0][1]), 3, (255, 0, 0))

                model_points = []
                image_points = []
                for match in good:
                    #print(self.kp1[match.imgIdx].pt, kp2[match.queryIdx].pt)
                    # Adjust model points to center at target
                    model_points.append([(self.kp1[match.queryIdx].pt[0] - 260.0) / 556.0, (self.kp1[match.queryIdx].pt[1] - 80.0) / 556.0, 0.0])
                    image_points.append([float(kp2[match.trainIdx].pt[0]), float(kp2[match.trainIdx].pt[1])])

                model_points = np.array(model_points)
                image_points = np.array(image_points).reshape((len(image_points),1,2))

                (success, rotation_vector, translation_vector, _) = cv.solvePnPRansac(
                   model_points, image_points, self.occam_camera_matrix, self.occam_distortion_coefs,
                   reprojectionError=5.0, flags=cv.SOLVEPNP_ITERATIVE)

                print("%2.2f %2.2f %2.2f" % (translation_vector[0], translation_vector[1], translation_vector[2]))

                # Torpedo starting point relative to camera
                torpedo_start = np.array([0.0, 0.07, 0.445], dtype=np.float)
                target = np.array([translation_vector[0][0], translation_vector[1], translation_vector[2]], dtype=np.float)

                # Project torped point distance to target
                torpedo_end = np.add(torpedo_start, np.array([0, 0, np.linalg.norm(target-torpedo_start)]))

                new_pts = np.vstack((torpedo_start, torpedo_end, target))

                ZEROSX3 = np.array([0, 0, 0], dtype=np.float)
                #new_pts = np.array(new_pts, dtype=np.float)
                #print(new_pts)
                image_pts = cv.projectPoints(new_pts, ZEROSX3, ZEROSX3, self.occam_camera_matrix, self.occam_distortion_coefs)[0]

                print(image_pts)

                cv.circle(img3, (int(image_pts[0][0][0]+w), int(image_pts[0][0][1])), 3, (255,255,0), 2)
                cv.circle(img3, (int(image_pts[1][0][0]+w), int(image_pts[1][0][1])), 3, (0,255,255), 2)
                cv.circle(img3, (int(image_pts[2][0][0]+w), int(image_pts[2][0][1])), 3, (0,0,255), 2)

                image_message = self.bridge.cv2_to_imgmsg(img3, encoding='rgb8')
                # image_message.header = image.header
                self.debug_pub.publish(image_message)

            else:

                CUPrint( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
                matchesMask = None
        #except Exception as e:
        #    print(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Homography_Sift_Matcher')
    HSM = HomographySiftMatcher()
    try:
        HSM.run()
    except rospy.ROSInterruptException:
        pass