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
from localizer.srv import ClassicalBoxes2Poses
from pdb import set_trace

SERVER_SUCCESS = 1
SERVER_FAILURE = 0

DEBUG=True

class Localizer():
    def __init__(self):
        self.darknet_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes,self.boxes_received, queue_size=1, buff_size=10000000)
        self.pose_pub = rospy.Publisher('cusub_perception/mapper/task_poses', Detection, queue_size=1)
        self.create_server_dict()
        rospy.loginfo("Localizer Client Initialized")

    def create_server_dict(self):
        """
        Find all servers that we may use and load them into our server dict
        """
        self.server_dict = {}
        tasks_param = rospy.search_param('mission_tasks')
        tasks_to_do = rospy.get_param(tasks_param)
        param_list = rospy.get_param_names()
        rospy.loginfo("Loading Params for Active Mission Tasks: " + str(tasks_to_do))
        rospy.sleep(0.1) # allow above line to print
        for task in tasks_to_do:

            # Search the parameter server for our object's localizers
            search_str = task + '/object_localizers'
            obj_params = [x for x in param_list if search_str in x]
            
            # Add classes to the dictionary
            for op in obj_params:
                index = op.rfind('/') # trim off the end of the param that holds the class
                servers = rospy.get_param(op)
                if type(servers) != list: # Make the servers a list before adding it
                    servers = [servers]    
                self.server_dict[op[index+1:]] = servers
        self.test_server_conn()
        
    def test_server_conn(self):
        req = ClassicalBoxes2Poses()

        for task_srvs in self.server_dict.keys():
                for srv in self.server_dict[task_srvs]:
                    rospy.wait_for_service('cusub_perception/localize_' + srv, 10.0)

    def check_box_capability(self, boxes):
        """
        Remove any boxes our config hasn't told us how to localize
        """
        new_boxes = []
        for box in boxes:
            if box.Class not in self.server_dict.keys():
                if DEBUG:
                    rospy.logwarn("No server given to client for: " + box.Class)
            else:
                new_boxes.append(box)
        return new_boxes
                
    # main callback
    def boxes_received(self,msg):
        """
        Find which servers we want to work with
        Make requests for them
        -> Feature: if items use the same server, make a service request with all of the boxes at once
        -> Feature: Have support for backup localizing servers
        """
        if DEBUG:
            class_list = []
            for box in msg.bounding_boxes:
                class_list.append(box.Class)
            rospy.loginfo("Received: " + str(class_list))
        
        msg.bounding_boxes = self.check_box_capability(msg.bounding_boxes)
        # Initialize all possible server requests
        server_reqs = {}
        for cls in self.server_dict.keys():
            for srv_name in self.server_dict[cls]:
                if srv_name not in server_reqs.keys():
                    server_reqs[srv_name] = ClassicalBoxes2Poses()
                    server_reqs[srv_name].image = msg.image
                    server_reqs[srv_name].boxes = []

        # Populate the server requests with bounding boxes
        for box in msg.bounding_boxes:
            """ Need to ignore boxes with points near edge of camera frame.  Could be partial detections with out of frame components. """
            if(box.xmin < 60 \
            or box.ymin < 60 \
            or box.xmax > msg.image.width - 60 \
            or box.ymax > msg.image.height - 60):
                continue
            for srv in self.server_dict[box.Class]:
                server_reqs[srv].boxes.append(box)


        classes_found = []
        # Initiate the service calls
        for box in msg.bounding_boxes: # Loop through boxes
            if box.Class == "Found": # if we've already localized it, skip it
                continue
            for srv in self.server_dict[box.Class]: # loop through the servers available to this box
                if server_reqs[srv] == SERVER_SUCCESS:
                    break
                elif server_reqs[srv] == SERVER_FAILURE: # go to the next server
                    continue
                else: # Make the request
                    try:
                        req = server_reqs[srv]
                        rospy.wait_for_service('cusub_perception/localize_' + srv)
                        handler = rospy.ServiceProxy('cusub_perception/localize_' + srv, req)
                        res = handler(req.image, req.boxes)
                        
                        # Mark the localized boxes as found so we don't try to localize them again
                        server_reqs[srv] = SERVER_FAILURE
                        for cls in res.classes:
                            for j in range(len(msg.bounding_boxes)):
                                if cls in msg.bounding_boxes[j].Class:
                                    msg.bounding_boxes[j].Class = "Found"
                                    server_reqs[srv] = SERVER_SUCCESS
                        if server_reqs[srv] == SERVER_FAILURE:
                            if DEBUG:
                                rospy.logwarn("server " + srv + " failed")
                        else:
                            for i in range(len(res.poses)):
                                if res.classes[i] not in classes_found:
                                    classes_found.append(res.classes[i])
                                else: # another server already found this class
                                    continue
                                if DEBUG:
                                    rospy.loginfo("Publishing "+res.classes[i]+" pose")
                                self.publish_pose(res.classes[i], msg.image_header.frame_id, msg.image_header, res.poses[i])
                    except Exception as e:
                        rospy.logerr(e)

    def publish_pose(self, object_type, camera_frame, image_header, pose):

        detection = Detection()
        detection.location = pose
        detection.object_type = object_type
        detection.camera_frame = camera_frame
        detection.image_header = image_header

        self.pose_pub.publish(detection)

def test():
    rospy.init_node('cusub_perception/localizer_client')
    l = Localizer()

    # Fake data
    bb = BoundingBoxes()
    bb.bounding_boxes = []
    bb.image = Image()
    
    b1 = BoundingBox()
    b1.Class = "start_gate_pole"
    
    b2 = BoundingBox()
    b2.Class = "dice1"
    
    b3 = BoundingBox()
    b3.Class = "dice2"

    b4 = BoundingBox()
    b4.Class = "random"
    
    bb.bounding_boxes.append(b1)
    bb.bounding_boxes.append(b2)
    bb.bounding_boxes.append(b3)
    bb.bounding_boxes.append(b4)
    l.boxes_received(bb)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()
    

def main():
    rospy.init_node('localizer_client')
    l = Localizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__=="__main__":
#    test()
    main()
