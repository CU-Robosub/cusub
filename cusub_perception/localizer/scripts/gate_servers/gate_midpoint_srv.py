#!/usr/bin/env python
from __future__ import division
"""
Classical CV Server for StartGate
Localizes the gate by declaring the gate as METERS_AHEAD in the x dir
Adjust the y position of the gate by taking the difference between the middle of the frame and middle of the gate
"""
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from localizer.srv import ClassicalBox2Pose
from geometry_msgs.msg import Pose

from pdb import set_trace

PIXEL_2_METER_MOVEMENT = 80
METERS_AHEAD = 6

class GateMidpointServer():

    def __init__(self):
        self.server = rospy.Service("/localize_start_gate_pole", ClassicalBox2Pose, self.localize)
        rospy.loginfo("Gate Midpoint Initialized")

    def _orderBoxesLeft2Right(self, box1, box2):
        if box1[0] < box2[0]:
            return box1, box2
        else:
            return box2, box1

    def _getCenter(self, bounding_box):
        center_x = int( (bounding_box.xmax + bounding_box.xmin) / 2)
        center_y = int( (bounding_box.ymax + bounding_box.ymin) / 2)
        return (center_x, center_y)

    def localize(self, req):
        """
        This function just finds how shifted to the right or left in front of the gate is, and says it's 5m ahead. Through this carrot in front of the sub technique we can center on the gate and go through it.
        """
        # calculate image dims
        bounding_boxes = req.boxes
        image_dims = (req.image.height, req.image.width)


        
        x_pos = METERS_AHEAD # always send the sub 6m ahead, carrot out in front
        # rest of this function is moving the carrot side to side
        
        box1 = bounding_boxes[0]
        box2 = bounding_boxes[1]
        center1 = self._getCenter(box1) # NOTE WE SWITCH X (the column) to be the first element in the list here
        center2 = self._getCenter(box2)

        leftBoxCenter, rightBoxCenter = self._orderBoxesLeft2Right(center1, center2)
        
        gateMidpt_y = int( (leftBoxCenter[0] + rightBoxCenter[0]) / 2 )

        difference_y = int(image_dims[1] / 2) - gateMidpt_y

        y_pos = difference_y / PIXEL_2_METER_MOVEMENT

        gateMidpt_z = int( (leftBoxCenter[1] + rightBoxCenter[1]) / 2 )
        difference_z = int(image_dims[0] / 2) - gateMidpt_z
        z_pos = -difference_z / PIXEL_2_METER_MOVEMENT # let's go a meter deeper

        pose = Pose()
        pose.position.x = x_pos
        pose.position.y = y_pos
        pose.position.z = z_pos
        
        return pose

def main():
    rospy.init_node('gate_localizer')
    server = GateMidpointServer()
    rospy.spin()

def test1():
    g = GateLocalizer()
    image_dims = (480,752)
    # image is 480x752

    # does x go along the rows or columns??
    
    boxes = []
    boxLeft = BoundingBox()
    boxLeft.xmin = 607
    boxLeft.ymin = 154
    boxLeft.xmax = 629
    boxLeft.ymax = 410

    boxRight = BoundingBox()
    boxRight.xmin = 206
    boxRight.ymin = 171
    boxRight.xmax = 231
    boxRight.ymax = 391
    
    boxes.append(boxLeft)
    boxes.append(boxRight)
    print(g.midpoint(boxes, image_dims))

if __name__ == "__main__":
    main()
#    test1()

    


    
    
