#!/usr/bin/env python
"""
Generates darknet training data from bounding boxes.  This allows you
to take simulated imagry.  Create bounding boxes on it autonomusly and
train a neural network on it potentially bypassing the tedious process
of hand labeling training data.
"""

import os

import rospy

from darknet_ros_msgs.msg import BoundingBoxes

from cv_bridge import CvBridge

import cv2

class TrainingDataGenerator(object):
    """Generates training data from bounding boxes
    """

    output_directory = None
    """str : name of directory to output training data to"""

    detection_classes = None
    """list : List of dictionarys giving name and number of detection"""

    bridge = None
    """cv_bridge.CvBridge: CV Bridge to transform between ROS and cv images"""

    image_count = 0
    """int : count of images we have processed"""

    def __init__(self):
        pass

    def bounding_boxes_callback(self, bounding_boxes):
        """Receives bounding box detections and outputs training data
        based on what is in them
        """

        det_strs = []

        w = float(bounding_boxes.image.width)
        h = float(bounding_boxes.image.height)

        # Find bounding box class matches and generate detection text for them
        for bounding_box in bounding_boxes.bounding_boxes:

            """ Need to ignore boxes with points near edge of camera frame.  Could be partial detections with out of frame components. """
            if(bounding_box.xmax < 60 \
            or bounding_box.ymax < 60 \
            or bounding_box.xmin > w - 60 \
            or bounding_box.ymin > h - 60):
                continue

            for detection_class in self.detection_classes:
                if bounding_box.Class == detection_class['name']:

                    # <object-class> <x> <y> <width> <height>
                    # <object-class> integer number of boject 0 to classes-1
                    # <x> <y> <width> <height> float values relative to width
                    #  and height of image 0.0 - 1.0
                    det_text = str(detection_class['number']) \
                             + ' ' + str((bounding_box.xmin + bounding_box.xmax) / 2.0 / w) \
                             + ' ' + str((bounding_box.ymin + bounding_box.ymax) / 2.0 / h) \
                             + ' ' + str((bounding_box.xmax - bounding_box.xmin) / w) \
                             + ' ' + str((bounding_box.ymax - bounding_box.ymin) / h)
                    det_strs.append(det_text)
                    break

        # write image file and detections

        filename = self.output_directory + ('/img/img%05d' % self.image_count)

        cv_img = self.bridge.imgmsg_to_cv2(bounding_boxes.image, "bgr8")
        cv2.imwrite(filename + '.jpg', cv_img)

        with open(filename + '.txt', 'w') as det_file:
            for det_str in det_strs:
                det_file.write(det_str + '\n')

        self.train_file.write('img/img%05d.jpg\n' % self.image_count)
        self.train_file.flush()

        self.image_count += 1

    def run(self):
        """Starts up node to create training data and then spins
        """

        self.bridge = CvBridge()

        self.output_directory = os.path.expanduser(rospy.get_param('~output_directory'))

        # Create directorys where image data will be saved if they dont exist
        try:
            os.makedirs(self.output_directory + '/img')
        except:
            pass

        self.train_file = open(self.output_directory + '/train.txt', 'w')

        bounding_boxes_topic = rospy.get_param('~bounding_boxes_topic')

        self.detection_classes = rospy.get_param('~classes')

        rospy.Subscriber(bounding_boxes_topic, BoundingBoxes,
                         self.bounding_boxes_callback, queue_size=1)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('Training Data Generator')
    TRAINER = TrainingDataGenerator() # Gotta catch um all
    try:
        TRAINER.run()
    except rospy.ROSInterruptException:
        rospy.logerr("YOLOFaker has died!")
