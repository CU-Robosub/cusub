#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from functools import partial
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class YOLOObjectClass(object):
    """
    Holds object information for a bounding box clase (i.e. the pole part of the startgate)
    This is for sub parts of objects that need to be classified rather than classifying the object as a whole
    """

    name = None
    color = None
    points = None

    def __init__(self, name, color, points):
        self.name = name
        self.color = color
        self.points = points

class YOLOObject(object):
    """
    Holds object information required for bounding box generation
    """

    frameID = None
    classes = None

    def __init__(self, frameID, classes):
        self.frameID = frameID
        self.classes = classes

class Camera(object):
    """
    Holds camera information required for bounding box generation
    """

    frame = None

    w = 0
    h = 0

    cameraMatrix = None
    distortionCoeff = None

    debugImageEnabled = False
    debugImagePub = None

    def __init__(self, frame, w=None, h=None, cameraMatrix=None, distortionCoeff=None, debugImageEnabled=False, debugImagePub=None):
        self.frame = frame
        self.w = w
        self.h = h
        self.cameraMatrix = cameraMatrix
        self.distortionCoeff = distortionCoeff
        self.debugImageEnabled = debugImageEnabled
        self.debugImagePub = debugImagePub

    def isReady(self):
        """
        If the camera parameters have been set returns true
        """
        if(self.cameraMatrix is not None and self.distortionCoeff is not None):
            return True
        return False

class YOLOFaker(object):
    """
    YOLO Faker Node
    Takes in configuration yaml for camera, robot, and obstacle configurations and generates bounding boxes
    Requires pose sensors on objects
    """

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
        """
        Callback to publish transforms for objects so we can transform the object points in to the camera frame later
        """

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, obj.frameID, "world")

    def robotOdometryCallback(self, odom):
        """
        Callback to broadcast ground truth of robot so we can transform into the real camera frame laster
        """

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, "leviathan_gt/base_link", "world")

        tvec = (0.055, 0, 0.22)
        rvec = (0, 0, 0, 1)
        self.br.sendTransform(tvec, rvec, odom.header.stamp, "leviathan_gt/occam0_frame", "leviathan_gt/base_link")

    def cameraInfoCallback(self, info, camera):
        """
        Get camera info for each camera we are running on for accurate point projection
        """

        if not camera.isReady():

            camera.cameraMatrix = np.array(info.K)
            camera.cameraMatrix.shape = (3, 3)
            camera.distortionCoeff = np.array(info.D)

            camera.w = info.width
            camera.h = info.height

    def cameraImageCallback(self, image, camera):
        """
        Compute where the bounding boxes should be in the image, and fake YOLO bounding boxes output
        We also draw the debug image potentialy
        """

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        if camera.isReady():

            # Fake darknet yolo detections message
            bbs = BoundingBoxes()

            bbs.header = image.header
            bbs.image_header = image.header
            bbs.image = image
            bbs.bounding_boxes = []

            for objName, obj in self.objects.iteritems():

                detections = self.getDetections(obj, camera)

                if detections is not None:

                    if camera.debugImageEnabled:

                        for box in detections:
                            cv2.rectangle(
                                cv_image,
                                (int(box[0][0]), int(box[0][2])),
                                (int(box[0][1]), int(box[0][3])),
                                box[1], 2
                                )

                    for det in detections:

                        bb = BoundingBox()

                        bb.Class = box[2]
                        bb.probability = 1.0
                        bb.xmin = int(det[0][0])
                        bb.xmax = int(det[0][1])
                        bb.ymin = int(det[0][2])
                        bb.ymax = int(det[0][3])
                        bbs.bounding_boxes.append(bb)

            # only publish detection if there are boxes in it
            if len(bbs.bounding_boxes) > 0:
                self.darknetDetectionPub.publish(bbs)


        if camera.debugImageEnabled:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
            camera.debugImagePub.publish(image_message)

    def getDetections(self, obj, camera):
        """
        Takes in object and camera and finds objects in the cameras view
        """

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

                # Filter objects when they get to narrow to realistically be detected
                if bb[1] - bb[0] > 10 and bb[3] - bb[2] > 10:
                    detections.append((bb, objClass.color, objClass.name))

        return detections

    def boundingBoxFromPoints(self, pts, camera):
        """
        Generate a bounding box from a set of image points
        """

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

        self.bridge = CvBridge()

        boudingBoxesTopic = rospy.get_param('~bounding_boxes_topic')
        subPoseTopic = rospy.get_param('~sub_pose_topic')

        # Fake darknet YOLO
        self.darknetDetectionPub = rospy.Publisher(boudingBoxesTopic, BoundingBoxes, queue_size=1)

        # Get robot pose so we can have its ground truth for transforms
        rospy.Subscriber(subPoseTopic, Odometry, self.robotOdometryCallback)

        # Setup cameras
        self.cameras = {}
        cameras = rospy.get_param('~cameras')
        for cameraname, camera in cameras.iteritems():

            self.cameras[cameraname] = Camera(camera['frame'], debugImageEnabled=camera['debug_image_enabled'])

            infocallback = partial(self.cameraInfoCallback, camera=self.cameras[cameraname])
            imagecallback = partial(self.cameraImageCallback, camera=self.cameras[cameraname])

            rospy.Subscriber(camera['camera_info_topic'], CameraInfo, infocallback)
            rospy.Subscriber(camera['image_topic'], Image, imagecallback)

            if(camera['debug_image_enabled']):
                self.cameras[cameraname].debugImagePub = rospy.Publisher(camera['debug_image_topic'], Image, queue_size=1)

        # Setup objects
        self.objects = {}
        objects = rospy.get_param('~objects')
        for objectname, obj in objects.iteritems():

            objectClasses = []

            for objClass in obj['classes']:
                objectClasses.append(
                    YOLOObjectClass(objClass['name'], objClass['debug_color'], np.array(objClass['points'], dtype=np.float)))

            self.objects[objectname] = YOLOObject(obj['link'], objectClasses)

            # Subscribe to get odometry to generate world transforms for points
            odomcallback = partial(self.objectOdomCallback, obj=self.objects[objectname])
            rospy.Subscriber(obj['odom_topic'], Odometry, odomcallback, queue_size=1)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('YOLOFaker')
    a = YOLOFaker()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("YOLOFaker has died!")
