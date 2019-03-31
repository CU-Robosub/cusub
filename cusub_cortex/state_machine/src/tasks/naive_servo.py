#!/usr/bin/env python
"""
Naive visual servoing tool
"""
import rospy
from std_msgs.msg import Float64
from topic_tools.srv import MuxSelect
from darknet_ros_msgs.msg import BoundingBoxes

OCCAM_IMAGE_MAX = 752
OCCAM_IMAGE_MIN = 0

class NaiveVisualServoTool():
    """
    Visual Servo a bounding box with bangbang control.
    """
    outcomes = ['success', 'aborted']
    target = None
    non_linearity_const = 100 # pixels

    def scale_yaw(self, box_center):
        """
        This function scales the yaw state in order to assist the yaw PID loop locking onto a target
        """
        scale = OCCAM_IMAGE_MAX / (2 * self.non_linearity_const)
        
        diff = box_center - self.goal_pixel_x
        new_center = int(self.goal_pixel_x + diff * scale)
        if new_center < OCCAM_IMAGE_MIN:
            new_center = OCCAM_IMAGE_MIN
        elif new_center > OCCAM_IMAGE_MAX:
            new_center = OCCAM_IMAGE_MAX
        return new_center

    def boxes_received(self, msg):
        """
        Get the position of the dice
        """
        if not self.controlling:
            return
        # TODO check that the frame of the image is correct
        
        probability = 0.0
        the_one_true_box = None
        
        for box in msg.bounding_boxes: # find most probable box
            if box.Class == self.target:
                if box.probability > probability:
                    probability = box.probability
                    the_one_true_box = box
                    
        if the_one_true_box is not None:
            # rospy.loginfo("Recieved a box!")

            x_center = Float64()
            x_center.data = (the_one_true_box.xmin + the_one_true_box.xmax) / 2.0
            # rospy.loginfo(x_center.data)
            x_center.data = self.scale_yaw(x_center.data)
            # rospy.loginfo(x_center.data)
            self.yaw_state_pub.publish(x_center)

            x_target = Float64()
            x_target.data = self.goal_pixel_x
            self.yaw_setpoint_pub.publish(x_target)

            if self.active:
                res = self.handling_function(msg.image, the_one_true_box)
                if res == 0: # Magic number to stop visual servoing
                    self.controlling = False
                    self.active = False
        # else:
        #     rospy.logwarn("No on true box....")    

            # y_center = Float64()
            # y_center.data = (the_one_true_box.ymin + the_one_true_box.ymax) / 2.0
            # self.diceDepthStatePub.publish(y_center)

            # y_target = Float64()
            # y_target.data = self.goal_pixel_y
            # self.diceDepthSetpointPub.publish(y_target)

    def __init__(self, target_class, handling_function, target_frame="occam0", goal_pixel_x=376.0, goal_pixel_y=300.0, lockon_time=1.0):
        # TODO make which axis controlled part of the init
        rospy.loginfo("--visual-servoing-init")
        self.active = False
        self.controlling = False
        self.target = target_class
        self.target_frame = target_frame
        self.goal_pixel_x = goal_pixel_x
        self.goal_pixel_y = goal_pixel_y
        self.ns = rospy.get_namespace()
        self.lockon_time = lockon_time
        self.handling_function = handling_function

        # self.robotname = rospy.get_param('~robotname')

        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)

        self.yaw_state_pub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/state', Float64, queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/setpoint', Float64, queue_size=1)

        self.yaw_select = rospy.ServiceProxy('cusub_common/motor_controllers/yaw_mux/select', MuxSelect)

    def control(self):
        """
        Switch to bangbang control and back
        """
        self.controlling = True        
        self.yaw_select('/' + self.ns + '/cusub_common/motor_controllers/cv/yaw/control_effort')
        rospy.loginfo("---locking on...")
        rospy.sleep(self.lockon_time)
        rospy.loginfo("---get wrecked")
        self.active = True

        while self.active and not rospy.is_shutdown():
            pass
        self.yaw_select('/' + self.ns + '/cusub_common/motor_controllers/pid/yaw/control_effort')

    def run(self):
        rospy.loginfo("---visual-servoing " + self.target)
        self.control()
