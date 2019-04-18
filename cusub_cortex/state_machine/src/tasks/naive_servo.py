#!/usr/bin/env python
"""
Naive visual servoing tool
"""
import rospy
from std_msgs.msg import Float64
from topic_tools.srv import MuxSelect
from darknet_ros_msgs.msg import BoundingBoxes

SERVO_NO_THROTTLE = 0
SERVO_THROTTLE_THRESH = 0
SERVO_THROTTLE = 700

class ServoAxisConfig():
    def __init__(self, axis, frame, goal_pixel=376, row=False, scale=100):
        """
        Makes a axis servo configuration
        
        Parameters
        ----------
        axis : str
             The desired axis to control
             'yaw', 'drive', 'strafe', or 'depth'
        frame : str
             The frame expected for the bounding box
             'occam0', or 'downcam'
        goal_pixel : int
             The pixel to center the bounding box on
             Occam has dimension 752 x 480, Center: (376, 240)
             Downcam dimension 1024 x 960, Center: (512, 480)
        row : bool
             Is the goal_pixel on a row or column?
        scale : float
             Number of pixels difference between state and setpoint when max throttle begins
        
        Returns
        -------
        ServoAxisConfig object
        """
        if axis != 'yaw' and axis != 'drive' and axis != 'strafe' and axis != 'depth':
            rospy.logerr("Bad axis given to axis config object")
            return
        
        self.axis = axis
        self.frame = frame
        self.goal_pixel = goal_pixel
        self.row = row
        self.scale = scale

    def get_dict(self):
        axis_dict = {}
        axis_dict = {}
        axis_dict['goal_pixel'] = self.goal_pixel
        axis_dict['row'] = self.row
        axis_dict['scale'] = self.scale
        return axis_dict

# TODO
# Fix the active vs controlling variable... which does which
# handling function should call deactivate to stop
# Numpy documentation especially here
        
class NaiveVisualServoTool():
    """
    Visual Servo a bounding box with bangbang control.
    """
    target = None

    def scale(self, state, goal_pixel, scale):
        """
        This function scales the state in order to assist the PID loop locking onto a target
        """
        scale = SERVO_THROTTLE / (2 * scale)
        
        diff = state - goal_pixel
        new_state = int(goal_pixel + diff * scale)
        if new_state < 0:
            new_state = 0
        elif new_state > 752:
            new_state = 752
        return new_state

    def boxes_received(self, msg):
        """
        Get the position of the dice
        """
        if not self.controlling:
            return

        # DO an if frame in msg.frame so that we can be lazy with specificying the frames :)
        
        probability = 0.0
        the_one_true_box = None
        
        for box in msg.bounding_boxes: # find most probable box
            if box.Class == self.target:
                if box.probability > probability:
                    probability = box.probability
                    the_one_true_box = box
                    
        if the_one_true_box is not None:
            # rospy.loginfo("Box Received.")
            # We have a box, let's see if its in a frame that we care about
            msg_frame = msg.image_header.frame_id
            for frame in self.axis:
                if frame not in msg_frame:
                    rospy.logwarn("Received unexpected frame: " + msg_frame)
                    continue
                for axis in self.axis[frame]: # Loop through all of axis we want to control with this frame
                    
                    ### Setpoint
                    setpoint = Float64()
                    setpoint.data = self.axis[frame][axis]['goal_pixel']
                    self.axis[frame][axis]['setpoint_pub'].publish(setpoint)

                    ### State
                    state = Float64()
                    if self.axis[frame][axis]['row'] == True:
                        state.data = (the_one_true_box.ymin + the_one_true_box.ymax) / 2.0
                        state.data = self.scale(state.data, self.axis[frame][axis]['goal_pixel'], self.axis[frame][axis]['scale'])
                    else: # Column
                        state.data = (the_one_true_box.xmin + the_one_true_box.xmax) / 2.0
                        state.data = self.scale(state.data, self.axis[frame][axis]['goal_pixel'], self.axis[frame][axis]['scale'])
                    self.axis[frame][axis]['state_pub'].publish(state)
                    
            if self.active:
                self.handling_function(msg.image, the_one_true_box)

    def __init__(self, target_class, handling_function, axis_configs, lockon_time=1.0):
        # TODO make which axis controlled part of the init
        rospy.loginfo("--visual-servoing-init")
        self.active = False
        self.controlling = False
        self.target = target_class
        self.ns = rospy.get_namespace()
        self.lockon_time = lockon_time
        self.handling_function = handling_function
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)        

        # Initialize axis controls
        self.axis = {}
        if type(axis_configs) != list: # make a list for compatability
            axis_configs = [axis_configs]
        for ac in axis_configs:
            frame = ac.frame
            axis = ac.axis
            if frame not in self.axis.keys():
                self.axis[frame] = {}
            self.axis[frame][axis] = ac.get_dict()
            self.axis[frame][axis]['state_pub'] = rospy.Publisher('cusub_common/motor_controllers/cv/'+axis+'/state', Float64, queue_size=1)
            self.axis[frame][axis]['setpoint_pub'] = rospy.Publisher('cusub_common/motor_controllers/cv/'+axis+'/setpoint', Float64, queue_size=1)
            self.axis[frame][axis]['mux_selector'] = rospy.ServiceProxy('cusub_common/motor_controllers/'+axis+'_mux/select', MuxSelect)

    def run(self):
        rospy.loginfo("---visual-servoing " + self.target)

        self.controlling = True
        for frame in self.axis:
            for axis in self.axis[frame]:
                rospy.loginfo("---visual servo controlling " + axis)
                res = self.axis[frame][axis]['mux_selector']('/' + self.ns + '/cusub_common/motor_controllers/cv/'+axis+'/control_effort')
                
        rospy.loginfo("---locking on...")
        rospy.sleep(self.lockon_time)
        rospy.loginfo("---get wrecked")
        self.active = True

        while self.active and not rospy.is_shutdown():
            pass
        for frame in self.axis:
            for axis in self.axis[frame]:
                rospy.loginfo("---visual servo releasing " + axis)
                self.axis[frame][axis]['mux_selector']('/' + self.ns + '/cusub_common/motor_controllers/pid/'+axis+'/control_effort')
        
    def deactivate(self):
        self.controlling = False
        self.active = False
