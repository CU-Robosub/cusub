#!/usr/bin/env python
"""
This module contains the YOLOFaker node.  It generates bounding boxes
for objects in the sub simulator by taking in information about the
camera and the objects bounding points.  It listens for the ground
truth poses from gazebo and broadcasts them as transforms in the world
frame letting it transform the points in the object frame to the world
frame to be projected in the camera frame.  This lets it come up with a
bounding box in the camera image mimicing what YOLO would would produce.
This allows code development on statemachine or classical CV algorithms
without the neural net being trained or without the use of a GPU.

To use this module you will need to provide a YAML configuration file
specifying which cameras you want to use it on, their parameters, and
which objects you want to have boxed.  Objects can have multiple
classes each with there own set of bounding boxes for bounding box
generation.  This allows for a single object to generate multiple hits
like for example a start gate could have bounding points for each of
its poles allowing them to be recognized individualy.  For an example
of the yaml file look at fakeyolo_default.yaml

To get the poses of the objects you want to track you will need to attach
pose sensors to them in gazebo.

If you are using distortion in gazebo it is important to note that due
to cropping of the barrel distortion your points will not be projected
correctly unless you adjust the cameras field of view bigger than it
actually is and then provide a camera matrix with the correct parameters
which gazebo will not publish.
"""

from functools import partial

import rospy
import tf

import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge

import cv2

ZEROSX3 = np.array([0, 0, 0], dtype=np.float)
""" numpy.array : Constant array of zeros, used as rotation & translation
vector for project points
"""

class YOLOObjectClass(object):
    """ Detection class object

    Holds object information for a bounding box class (i.e. the pole
    part of the startgate).  This is for parts of objects that need
    to be classified rather than classifying the object as a whole.
    """

    def __init__(self, name, color, points):
        """
        Parameters
        ----------
        name : str
            The name of the detection
        color : tuple
            (R, G, B) tuple of color to draw detection on debug image
        points : numpy.array
            List of points bounding the volume of the object to generate
            a detection for
        """
        self._name = name
        self._color = color
        self._points = points

    @property
    def name(self):
        """Name of the object class

        Returns
        -------
        str
            Name of detection
        """
        return self._name

    @property
    def points(self):
        """Points representing bounding volume

        Returns
        -------
        numpy.array
            The points that define the bounding volume of the
            class relative to the object link
        """
        return self._points

    @property
    def color(self):
        """ Debug image color

        Returns
        -------
        tuple
            Color to use for debug image of detections of class
        """
        return self._color

class YOLOObject(object):
    """Holds object information required for bounding box generation"""

    def __init__(self, frame_id, classes):
        """
        Parameters
        ----------
        frame_id : str
            Frame id the object is referenced too, usualy baselink
        classes : list
            List of classes object can have detected in it
        """
        self._frame_id = frame_id
        self._classes = classes

    @property
    def frame_id(self):
        """ Frame ID

        Returns
        -------
        str
            Frame ID the object is referenced to
        """
        return self._frame_id

    @property
    def classes(self):
        """Detection Classes

        Returns
        -------
        list
            Classes of detections the object generates
        """
        return self._classes

class Camera(object):
    """
    Holds camera information required for bounding box generation
    """

    width = 0
    """int : The width of the image from the camera"""
    height = 0
    """int : The height of the image from the camera"""

    camera_matrix = None
    """numpy.array : Intrinsic 3x3 camera matrix as numpy array

    .. math::

        M = \\begin{bmatrix}
                f_{x} &     0 & c_{x} \\\\
                    0 & f_{y} & c_{y} \\\\
                    0 &     0 &     1
            \\end{bmatrix}

    """
    distortion_coeff = None
    """numpy.array : The distortion parameters for the "Plumb Bob" (Brown) model
    as numpy array (k1, k2, t1, t2, k3)
    """

    debug_image_pub = None
    """rospy.Publisher : ROS Publisher to send the debug images with bounding boxes drawn
    on them
    """

    def __init__(self, frame):
        """
        Parameters
        ----------
        frame : str
            Name of frame the camera is centered at
        """
        self._frame = frame

    @property
    def frame(self):
        """ Camera frame

        Returns
        -------
        str
            The name of the frame that the camera is in
        """
        return self._frame

    def is_ready(self):
        """Is the camera ready?

        Checks to see if the camera is ready by seeing if the object
        contiains the camera matrix and distortion coefficents needed
        to projection points into the image.

        Returns
        -------
        bool
            Camera readyness
        """
        if(self.camera_matrix is not None and self.distortion_coeff is not None):
            return True
        return False

class YOLOFaker(object):
    """YOLO Faker Node

    Takes in configuration yaml for camera, robot, and obstacle
    configurations and generates bounding boxes
    """
    # pylint: disable=too-many-instance-attributes
    # This is a node that needs a lot of parameters

    objects = {}
    """dict : Dictionary of objects used for bounding box generation"""

    cameras = {}
    """dict : Dictionary of cameras we generate detections for"""

    listener = None
    """tf.TransformListener : TF Listener so we can transform object points into the camera frame"""
    broadcaster = None
    """tf.TransformBroadcaster : TF Broadcaster so we can have transforms from the object
    ground truth pose to the world frame
    """
    bridge = None
    """cv_bridge.CvBridge: CV Bridge to transform between ROS and cv images"""

    darknet_detection_pub = None
    """rospy.Publisher : Publisher to send detections to"""

    min_pixel_size = 10
    """int: Minimum width or height of bounding box in pixels that will
    be shown as a detection
    """

    show_points = False
    """bool : Show debug bounding volume points"""

    def __init__(self):
        pass

    def object_odom_callback(self, odom, obj):
        """ Object odometry

        Callback to publish transforms for objects so we can transform
        the object points in to the camera frame later

        Parameters
        ----------
        odom : nav_msgs.msg.Odometry
            Current pose of object
        obj : YOLOObject
            Object we are getting odometry for
        """

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.broadcaster.sendTransform(tvec, rvec, odom.header.stamp, obj.frame_id, "world")

    def robot_odometry_callback(self, odom):
        """Robot odometry

        Callback to broadcast ground truth of robot so we can transform
        into the real camera frame

        Parameters
        ----------
        odom : nav_msgs.msg.Odometry
            Current pose of robot
        """

        tvec = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
        rvec = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.broadcaster.sendTransform(tvec, rvec, odom.header.stamp,
                                       "leviathan_gt/base_link", "world")

    @staticmethod
    def camera_info_callback(info, camera):
        """Get camera info for each camera we are running on for accurate point projection

        We need the camera information to project world space points into the image.
        To do this we take the width, height, intrinsic matrix, and distortion coefficients
        from the image message and put them in the camera object so when we get an image
        from that camera we know how to project the points.

        Parameters
        ----------
        info : sensor_msgs.CameraInfo
            CameraInfo message from ROS.  Contains camera parameters.
        camera : Camera
            Holds camera parameters for projecting points when we get an image.

        """

        if not camera.is_ready():

            camera.camera_matrix = np.array(info.K)
            camera.camera_matrix.shape = (3, 3)
            camera.distortion_coeff = np.array(info.D)

            camera.width = info.width
            camera.height = info.height

    def render_debug_context(self, cv_image, detections, camera):
        """Renders debug information to image

        Parameters
        ----------
        cv_image : numpy.ndarray
            Image to render detections on
        detections : tuple
            Detection data to render for debug
        show_points : bool
            Render bounding volume points?
        """

        for det in detections:

            # Draw bounding box
            cv2.rectangle(
                cv_image,
                (int(det[0][0]), int(det[0][2])),
                (int(det[0][1]), int(det[0][3])),
                det[1], 2
                )

            # Draw bounding volume points
            if self.show_points:
                for point in det[3]:
                    cv2.circle(
                        cv_image,
                        (int(point[0][0]), camera.height - int(point[0][1])),
                        3,
                        det[1],
                        2
                        )

    def camera_image_callback(self, image, camera):
        """Gets images from camera to generate detections on

        Computes where the bounding boxes should be in the image, and
        fakes YOLO bounding boxes output as well as publishing a debug
        image showing where the detections are if turned on

        Parameters
        ----------
        image : sensor_msgs.Image
            The image from the camera to create detections for
        camera : Camera
            Holds camera parameters for projecting points

        """

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        if camera.is_ready():

            # Fake darknet yolo detections message
            bounding_boxes = BoundingBoxes()

            bounding_boxes.header = image.header
            bounding_boxes.image_header = image.header
            bounding_boxes.image = image
            bounding_boxes.bounding_boxes = []

            for _, obj in self.objects.iteritems():

                detections = self.get_detections(obj, camera, image.header.stamp)

                if detections is not None:

                    if camera.debug_image_pub:

                        self.render_debug_context(cv_image, detections, camera)

                    for det in detections:

                        bounding_box = BoundingBox()

                        bounding_box.Class = det[2]
                        bounding_box.probability = 1.0
                        bounding_box.xmin = int(det[0][0])
                        bounding_box.xmax = int(det[0][1])
                        bounding_box.ymin = int(det[0][2])
                        bounding_box.ymax = int(det[0][3])
                        bounding_boxes.bounding_boxes.append(bounding_box)

            # Only publish detection if there are boxes in it
            if bounding_boxes.bounding_boxes:
                self.darknet_detection_pub.publish(bounding_boxes)

        if camera.debug_image_pub:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
            camera.debug_image_pub.publish(image_message)

    def get_detections(self, obj, camera, time):
        """Takes in object and camera and finds objects in the cameras view

        Parameters
        ----------
        obj : YOLOObject
            Object containg detection classes to find in image space
        camera : Camera
            Camera parameters to use when projecting object points into image space
        time : Time
            Time to get detections for

        Returns
        -------
        list
            List of detections (Bounding Box, Color, Name, Points) in current camera image
        """

        detections = []

        for obj_class in obj.classes:

            # check that we are not beyond viewing distance
            self.listener.waitForTransform(camera.frame, obj.frame_id, time, rospy.Duration(1))
            (obj_trans, obj_rot) = self.listener.lookupTransform(camera.frame, obj.frame_id, time)
            if np.linalg.norm(obj_trans) > rospy.get_param('yolo_faker/visibility', 14.0):
                continue

            new_pts = []
            for point in obj_class.points:

                obj_pose_stamped = PoseStamped()
                obj_pose_stamped.header.frame_id = obj.frame_id
                obj_pose_stamped.header.stamp = time
                pose = Pose()
                position = Point()
                position.x = point[0]
                position.y = point[1]
                position.z = point[2]
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
                image_pts = cv2.projectPoints(new_pts, ZEROSX3, ZEROSX3,
                                              camera.camera_matrix, camera.distortion_coeff)

                bounding_box = self.bounding_box_from_points(image_pts[0], camera)

                # Filter objects when they get to narrow to realistically be detected
                if bounding_box[1] - bounding_box[0] > self.min_pixel_size \
                   and bounding_box[3] - bounding_box[2] > self.min_pixel_size:
                    detections.append((bounding_box, obj_class.color,
                                       obj_class.name, image_pts[0]))

            # TODO Find bounding boxes and if overlap is to great remove the further bounding box

        return detections

    @staticmethod
    def bounding_box_from_points(points, camera):
        """Generate a bounding box from a set of image points

        Parameters
        ----------
        points : numpy.array
            3D points bounding object volume
        camera : Camera
            Informatinon used for projecting points into image

        Returns
        -------
        tuple
            Tuple bounding box (xmin, xmax, ymin, ymax)

        """

        xmin = camera.width
        xmax = 0

        ymin = camera.height
        ymax = 0

        for point in points:

            pnt_x = point[0][0]
            pnt_y = camera.height - point[0][1] # y axis inverted in image space

            xmin = min(pnt_x, xmin)
            xmax = max(pnt_x, xmax)
            ymin = min(pnt_y, ymin)
            ymax = max(pnt_y, ymax)

        xmin = max(xmin, 0)
        xmax = min(xmax, camera.width)
        ymin = max(ymin, 0)
        ymax = min(ymax, camera.height)

        return (xmin, xmax, ymin, ymax)

    def run(self):
        """
        Starts the yolo_faker node by initalizing all the subscribers
        and publishers.  Spins for rest of nodes life.
        """

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.bridge = CvBridge()

        # Topic to publish fake detections to
        bouding_boxes_topic = rospy.get_param('yolo_faker/bounding_boxes_topic')
        sub_pose_topic = rospy.get_param('yolo_faker/sub_pose_topic')

        # Parameter for smallest width/height allowed for bounding box in pixels
        self.min_pixel_size = rospy.get_param('yolo_faker/min_pixel_size', 15)

        # Shows bounding volume points for debugging
        self.show_points = rospy.get_param('~show_points', False)

        # Fake darknet YOLO
        self.darknet_detection_pub = rospy.Publisher(bouding_boxes_topic,
                                                     BoundingBoxes, queue_size=1)

        # Get robot pose so we can have its ground truth for transforms
        rospy.Subscriber(sub_pose_topic, Odometry, self.robot_odometry_callback)

        # Setup cameras node will generate detections for
        self.cameras = {}
        cameras = rospy.get_param('yolo_faker/cameras')
        for cameraname, camera in cameras.iteritems():

            # We get camera info from the camera parameters publisher.
            # We need the camera matrix, distortion coefficents, width,
            # and height to work correctly.  The code waits for these
            # parameters before it starts creating fake detections.

            # The bounding boxes are generated when an image comes in
            # from the camera.

            self.cameras[cameraname] = Camera(camera['frame'])

            infocallback = partial(self.camera_info_callback, camera=self.cameras[cameraname])
            imagecallback = partial(self.camera_image_callback, camera=self.cameras[cameraname])

            rospy.Subscriber(camera['camera_info_topic'], CameraInfo, infocallback)
            rospy.Subscriber(camera['image_topic'], Image, imagecallback)

            if camera['debug_image_enabled']:
                self.cameras[cameraname].debug_image_pub = \
                  rospy.Publisher(camera['debug_image_topic'], Image, queue_size=1)

        # Setup objects that will generate bounding boxes
        self.objects = {}
        objects = rospy.get_param('yolo_faker/objects')
        for objectname, obj in objects.iteritems():

            # Objects are defined as a set of classes each with a set of
            # bounding points that allow for an object to generate
            # detections on different unique parts of itself.

            # Each object has specifies a link in the model that the points
            # are referenced to.

            object_classes = []

            for obj_class in obj['classes']:
                object_classes.append(
                    YOLOObjectClass(obj_class['name'], obj_class['debug_color'],
                                    np.array(obj_class['points'], dtype=np.float)))

            self.objects[objectname] = YOLOObject(obj['link'], object_classes)

            # Subscribe to get odometry to generate world transforms for points
            odomcallback = partial(self.object_odom_callback, obj=self.objects[objectname])
            rospy.Subscriber(obj['odom_topic'], Odometry, odomcallback, queue_size=1)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('YOLOFaker')
    YOLOFAKER = YOLOFaker()
    try:
        YOLOFAKER.run()
    except rospy.ROSInterruptException:
        rospy.logerr("YOLOFaker has died!")
