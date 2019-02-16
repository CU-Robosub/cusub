#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError

class YOLOObjectClass():

    name = None
    color = None
    points = None

    def __init__(self, name, color, points):
        self.name = name
        self.color = color
        self.points = points

class YOLOObject():

    frameID = None
    classes = None

    def __init__(self, frameID, classes):
        self.frameID = frameID
        self.classes = classes

class Camera():

    frame = None

    w = 0
    h = 0

    cameraMatrix = None
    distortionCoeff = None

    def __init__(self, frame, w, h, cameraMatrix, distortionCoeff):
        self.frame = frame
        self.w = w
        self.h = h
        self.cameraMatrix = cameraMatrix
        self.distortionCoeff = distortionCoeff

class YOLOFaker():

    # Constants, used everytime we do project points
    rvec = np.array([0, 0, 0], dtype=np.float)
    tvec = np.array([0, 0, 0], dtype=np.float)

    # List of objects being tracked
    objects = {}

    # List of cameras tracking
    cameras = {}

    def __init__(self):
        pass

    def objectOdomCallback(self, odom, obj):

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, obj.frameID, "world")

    def leviathanOdometryCallback(self, odom):

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, "leviathan_gt/base_link", "world")

        tvec = (0.055, 0, 0.22)
        rvec = (0, 0, 0, 1)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, "leviathan_gt/occam0_frame", "leviathan_gt/base_link")

    def occam0Image(self, image):

        # TODO lambda to spec camera in main loop
        camera = self.cameras['occam0']

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        for objName, obj in self.objects.iteritems():

            detections = self.getDetections(obj, camera)

            if detections is not None:
                for box in detections:
                    cv2.rectangle(
                        cv_image,
                        (int(box[0][0]), int(box[0][2])),
                        (int(box[0][1]), int(box[0][3])),
                        box[1], 2
                        )

        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")

        self.occam0_image_pub.publish(image_message)

    def getDetections(self, obj, camera):

        detections = []

        for objClass in obj.classes:

            new_pts = []
            for pt in objClass.points:

                obj_pose_stamped = PoseStamped()
                obj_pose_stamped.header.frame_id = obj.frameID
                pose = Pose()
                position = Point()
                position.x = pt[0]
                position.y = pt[1]
                position.z = pt[2]
                orientation = Quaternion()
                orientation.x = 0
                orientation.y = 0
                orientation.z = 0
                orientation.w = 0
                pose.position = position
                pose.orientation = orientation
                obj_pose_stamped.pose = pose

                new_pt = self.listener.transformPose(camera.frame, obj_pose_stamped)
                new_pt = [-1*new_pt.pose.position.y, new_pt.pose.position.z, new_pt.pose.position.x]
                new_pts.append(new_pt)

                # Check if point is behind us.  If it is no detection
                if new_pt[2] < 0.0:
                    new_pts = None
                    break

            if new_pts is not None:

                new_pts = np.array(new_pts, dtype=np.float)
                result = cv2.projectPoints(new_pts, self.rvec, self.tvec, camera.cameraMatrix, camera.distortionCoeff)

                bb = self.boundingBoxFromPoints(result[0], camera)

                if bb[1] - bb[0] > 10 and bb[3] - bb[2] > 10:
                    detections.append((bb, objClass.color))

        return detections

    def boundingBoxFromPoints(self, pts, camera):

        xmin = camera.w
        xmax = 0

        ymin = camera.h
        ymax = 0

        for pt in pts:

            x = pt[0][0]
            y = pt[0][1]

            y = camera.h - y

            xmin = min(x, xmin)
            xmax = max(x, xmax)        
            ymin = min(y, ymin)
            ymax = max(y, ymax)

        xmin = max(xmin,   0)
        xmax = min(xmax, camera.w)
        ymin = max(ymin,   0)
        ymax = min(ymax, camera.h)

        return (xmin, xmax, ymin, ymax)

    def run(self):

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.pose_leviathan = rospy.Subscriber("/leviathan/pose_gt", Odometry, self.leviathanOdometryCallback)

        self.yaw_cv_ce_pub = rospy.Publisher('/leviathan/local_control/cv/yaw/control_effort', Float64, queue_size=1)

        self.bridge = CvBridge()

        self.occam0_image = rospy.Subscriber('/occam/image0', Image, self.occam0Image)
        self.occam0_image_pub = rospy.Publisher('/occam/image0_context', Image)

        """
        Set up cameras and objects
        TODO move to yaml
        """

        dice5Classes = [
            YOLOObjectClass(
                'dice',
                (255, 0, 0),
                np.array(
                    [
                        [ 0.0000, -0.2286,  0.0000],
                        [ 0.2286, -0.2286,  0.0000],
                        [ 0.0000,  0.0000,  0.0000],
                        [ 0.2286,  0.0000,  0.0000],
                        [ 0.0000, -0.2286, -0.2286],
                        [ 0.2286, -0.2286, -0.2286],
                        [ 0.0000,  0.0000, -0.2286],
                        [ 0.2286,  0.0000, -0.2286]
                    ], dtype=np.float)
                )
        ]
        dice6Classes = [
            YOLOObjectClass(
                'dice',
                (255, 255, 0),
                np.array(
                    [
                        [ 0.0000, -0.2286,  0.0000],
                        [ 0.2286, -0.2286,  0.0000],
                        [ 0.0000,  0.0000,  0.0000],
                        [ 0.2286,  0.0000,  0.0000],
                        [ 0.0000, -0.2286, -0.2286],
                        [ 0.2286, -0.2286, -0.2286],
                        [ 0.0000,  0.0000, -0.2286],
                        [ 0.2286,  0.0000, -0.2286]
                    ], dtype=np.float)
                )
        ]
        startgateClasses = [
            YOLOObjectClass(
                'start_gate_pole',
                (0, 255, 0),
                np.array(
                    [
                        [ 1.500, -0.075,  0.000],
                        [ 1.600, -0.075,  0.000],
                        [ 1.500,  0.075,  0.000],
                        [ 1.650,  0.075,  0.000],
                        [ 1.550, -0.075, -1.500],
                        [ 1.650, -0.075, -1.500],
                        [ 1.550,  0.075, -1.500],
                        [ 1.650,  0.075, -1.500]
                    ], dtype=np.float)
                ),
            YOLOObjectClass(
                'start_gate_pole',
                (0, 255, 0),
                np.array(
                    [
                        [-1.500, -0.075,  0.000],
                        [-1.600, -0.075,  0.000],
                        [-1.500,  0.075,  0.000],
                        [-1.650,  0.075,  0.000],
                        [-1.550, -0.075, -1.500],
                        [-1.650, -0.075, -1.500],
                        [-1.550,  0.075, -1.500],
                        [-1.650,  0.075, -1.500]
                    ], dtype=np.float)
                )
        ]

        self.objects = {}

        self.objects['dice5'] = YOLOObject('Dice5_1/base_link', dice5Classes)
        self.objects['dice6'] = YOLOObject('Dice6_1/base_link', dice6Classes)
        self.objects['startgate'] = YOLOObject('StartGate_1/base_link', startgateClasses)

        self.pose_dice5 = rospy.Subscriber("/Dice5_1/pose_gt", Odometry, lambda odom : self.objectOdomCallback(odom, self.objects['dice5']), queue_size=1)
        self.pose_dice6 = rospy.Subscriber("/Dice6_1/pose_gt", Odometry, lambda odom : self.objectOdomCallback(odom, self.objects['dice6']), queue_size=1)
        self.pose_dice6 = rospy.Subscriber("/StartGate_1/pose_gt", Odometry, lambda odom : self.objectOdomCallback(odom, self.objects['startgate']), queue_size=1)

        # TODO get from occam camera parameters publishing node
        occam_camera_matrix = np.array([656.926137, 0.000000, 376.5, 0.000000, 656.926137, 240.5, 0.000000, 0.000000, 1.000000], dtype=np.float)
        occam_camera_matrix.shape = (3,3)
        occam_distortion_coefs = np.array([-0.360085, 0.115116, 0.007768, 0.004616, 0.000000], dtype=np.float)

        self.cameras = {}
        self.cameras['occam0'] = Camera('leviathan_gt/occam0_frame', 752, 480, occam_camera_matrix, occam_distortion_coefs)


        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('YOLOFaker')
    a = YOLOFaker()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("YOLOFaker has died!");
