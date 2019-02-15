#!/usr/bin/python
"""
Localizer Client
The job of this node is the following:
-> Receive a box from darknet and determine its class
-> Call the correct localizing server to generate a pose from the box
-> Send the relative pose to the mapper to transform into a global pose

All servers requests have the naming convention of /localize_<class>
Ie for a class "start_gate_pole" the service would be /localize_start_gate_pole
"""

import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from localizer.msg import Detection
from localizer.srv import ClassicalBox2Pose


class Localizer():
    def __init__(self):
        self.darknet_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)
        self.pose_pub = rospy.Publisher('/Global_State/task_poses', Detection, queue_size=1)
        rospy.loginfo("Localizer Client Initialized")

    # main callback
    def boxes_received(self,msg):
        """
        - Loop through all of the boxes in the msg
        - At each box, determine the class, perform necessary logic
        - Make a service call to generate a pose from each box
        """

        # initialize tmp variables for multi object service calls
        sg_pole1 = None

        for box in msg.bounding_boxes: # loop through all of the boxes

            """ Need to ignore boxes with points near edge of camera frame.  Could be partial detections with out of frame components. """
            if(box.xmin < 60 \
            or box.ymin < 60 \
            or box.xmax > msg.image.width - 60 \
            or box.ymax > msg.image.height - 60):
                continue

            topic_name = box.Class # default

            # initalize the request
            req = ClassicalBox2Pose()
            req.image = msg.image
            req.boxes = [] ; req.boxes.append(box)

            # Perform some special logic for certain objects before localizing
            if box.Class == "start_gate_pole":
                """ we're looping through all of the boxes so record the first pole and then wait for the 2nd pole before we make the service call"""
                if sg_pole1 is None:
                    sg_pole1 = box
                    continue
                else: # now that we have the 2nd box, append it and allow the call to happen
                    req.boxes.append(sg_pole1)
                    topic_name = "start_gate"
            elif box.Class == "start_gate_flag":
                continue # don't use the flag

            # Make the service call
            try:
                rospy.loginfo("Waiting for service: /localize_"+box.Class)
                localize_handler = rospy.ServiceProxy('/localize_'+box.Class, req)
                res = localize_handler(req.image, req.boxes)
                rospy.loginfo("Publishing to {}".format(topic_name))
                self.publish_pose(topic_name, msg.image_header.frame_id, msg.image_header, res.pose)
            except Exception as e:
                rospy.logerr(e)

    def publish_pose(self, object_type, camera_frame, image_header, pose):

        detection = Detection()
        detection.location = pose
        detection.object_type = object_type
        detection.camera_frame = camera_frame
        detection.image_header = image_header

        self.pose_pub.publish(detection)

def main():
    rospy.init_node('localizer_client')
    l = Localizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__=="__main__":
    main()

# TODO load the class names to the localizer via rosparams, initialize the server handlers using the rosparams and store in an internal dictionary, when looping through the boxes call that handler in the dictionary instead of creating the handler here everytime
