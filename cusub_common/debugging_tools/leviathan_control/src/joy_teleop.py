#!/usr/bin/env python
"""
Allows joystick teleoperation of submarine
"""

import math

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

from actuator.srv import ActivateActuator
from waypoint_navigator.srv import ToggleControl

class JoyTeleop(object):

    strafe_axes  = 0
    drive_axes = 1
    yaw_axes = 2
    depth_axes = 3
    pitch_axes = 2
    roll_axes = 2
    gripper_axes = 5

    default_depth = 0

    strafe_val  = 0.0
    drive_val = 0.0
    yaw_val = 0.0
    depth_val = 0.0
    pitch_val = 0.0
    roll_val = 0.0
    gripper_val = 1.0

    depth_f64 = None

    twist_effort    = 1.0
    translate_effort = 1.0

    thruster_power = 100.0

    yaw_sensitivity = 0.02
    drive_sensitivity = 0.02
    strafe_sensitivity = 0.02
    depth_sensitivity = 0.05

    left_dropper_triggered = False
    right_dropper_triggered = False
    left_torpedo_triggered = False
    right_torpedo_triggered = False

    left_dropper_avail = True
    right_dropper_avail = True
    left_torpedo_avail = True
    right_torpedo_avail = True

    current_yaw_updated = False
    current_drive_updated = False
    current_strafe_updated = False
    current_depth_updated = False

    joystick_in_control = True

    def toggle_waypoint_navigator(self):

        toggle_waypoint = rospy.ServiceProxy('toggleWaypointControl', ToggleControl)

        if self.joystick_in_control:

            # Don't give back control, let state machine do this
            # toggle_waypoint(True)

            # Keep track of state
            self.joystick_in_control = False

        else:

            # Stop waypoint navigator
            toggle_waypoint(False)

            # Reset setpoint to current position
            self.current_yaw_updated = False
            self.current_drive_updated = False
            self.current_strafe_updated = False
            self.current_depth_updated = False

            self.joystick_in_control = True

    def joystick_state(self, data):
        """Gets joystick data to figure out what to have the sub do
        """

        if data.buttons[0] and self.right_dropper_avail:
            self.right_dropper_triggered = True
        if data.buttons[1] and self.left_dropper_avail:
            self.left_dropper_triggered = True
        if data.buttons[4] and self.left_torpedo_avail:
            self.left_torpedo_triggered = True
        if data.buttons[5] and self.right_torpedo_avail:
            self.right_torpedo_triggered = True

        if data.buttons[2]:
            self.toggle_waypoint_navigator()

        self.strafe_val  = data.axes[self.strafe_axes]
        self.drive_val = data.axes[self.drive_axes]
        self.yaw_val = data.axes[self.yaw_axes]
        self.depth_val = data.axes[self.depth_axes]
        self.pitch_val = data.axes[self.pitch_axes]
        self.roll_val = data.axes[self.roll_axes]
        self.gripper_val = data.axes[self.gripper_axes]

    def yawStateCallback(self, data):
        if not self.current_yaw_updated:
            self.yaw_f64.data = data.data
            self.current_yaw_updated = True

    def driveStateCallback(self, data):
        if not self.current_drive_updated:
            self.drive_f64.data = data.data
            self.current_drive_updated = True

    def strafeStateCallback(self, data):
        if not self.current_strafe_updated:
            self.strafe_f64.data = data.data
            self.current_strafe_updated = True

    def depthStateCallback(self, data):
        if not self.current_depth_updated:
            self.depth_f64.data = data.data
            self.current_depth_updated = True

    @staticmethod
    def actuate(num, time):
        try:
            activate_actuator = rospy.ServiceProxy('activateActuator', ActivateActuator)
            activate_actuator(num, time)
        except:
            print "Actuation Failed"

    def actuate_left_dropper(self, _):
        self.actuate(0, 500)
        self.left_dropper_avail = True
    
    def actuate_right_dropper(self, _):
        self.actuate(1, 500)
        self.right_dropper_avail = True

    def actuate_left_torpedo(self, _):
        self.actuate(2, 500)
        self.left_torpedo_avail = True

    def actuate_right_torpedo(self, _):
        self.actuate(3, 500)
        self.right_torpedo_avail = True

    def joy_teleop(self):

        rospy.init_node('joy_teleop', anonymous=True)

        setpoint = rospy.get_param('~setpoint', True)

        self.yaw_f64 = Float64()
        self.yaw_f64.data = 0.0
        self.drive_f64 = Float64()
        self.drive_f64.data = 0.0
        self.strafe_f64 = Float64()
        self.strafe_f64.data = 0.0
        depth_f64 = Float64()
        depth_f64.data = 0.0

        if setpoint:

            pub_yaw = rospy.Publisher('motor_controllers/pid/yaw/setpoint',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('motor_controllers/pid/drive/setpoint',
                                        Float64, queue_size=10)
            pub_strafe = rospy.Publisher('motor_controllers/pid/strafe/setpoint',
                                         Float64, queue_size=10)

            # get current accumulated strafe and drive to use for adjustments
            rospy.Subscriber("motor_controllers/pid/yaw/state",
                             Float64, self.yawStateCallback)
            rospy.Subscriber("motor_controllers/pid/drive/state",
                             Float64, self.driveStateCallback)
            rospy.Subscriber("motor_controllers/pid/strafe/state",
                             Float64, self.strafeStateCallback)

        else: 

            pub_yaw = rospy.Publisher('motor_controllers/mux/yaw/control_effort',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('motor_controllers/mux/drive/control_effort',
                                         Float64, queue_size=10)
            pub_strafe = rospy.Publisher('motor_controllers/mux/strafe/control_effort',
                                         Float64, queue_size=10)

        pub_pitch = rospy.Publisher('motor_controllers/pid/pitch/setpoint',
                                    Float64, queue_size=10)
        pub_roll = rospy.Publisher('motor_controllers/pid/roll/setpoint',
                                   Float64, queue_size=10)

        pub_depth = rospy.Publisher('motor_controllers/pid/depth/setpoint',
                                    Float64, queue_size=10)

        gripper_outer_pub = rospy.Publisher('/leviathan/description/outer_controller/command', Float64, queue_size=1)
        gripper_inner_pub = rospy.Publisher('/leviathan/description/inner_controller/command', Float64, queue_size=1)

        self.strafe_axes = rospy.get_param("~strafe_axes", self.strafe_axes)
        self.drive_axes = rospy.get_param("~drive_axes", self.drive_axes)
        self.yaw_axes = rospy.get_param("~yaw_axes", self.yaw_axes)
        self.depth_axes = rospy.get_param("~depth_axes", self.depth_axes)
        self.pitch_axes = rospy.get_param("~pitch_axes", self.pitch_axes)
        self.roll_axes = rospy.get_param("~roll_axes", self.roll_axes)

        # Changed to use current depth
        # self.default_depth = rospy.get_param("~default_depth", self.default_depth)
        # depth_f64 = Float64()
        # depth_f64.data = self.default_depth

        pitch_f64 = Float64()
        pitch_f64.data = 0
        roll_f64 = Float64()
        roll_f64.data = 0

        rospy.Subscriber("joy", Joy, self.joystick_state)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            if self.joystick_in_control:

                if setpoint:

                    self.yaw_f64.data = self.yaw_f64.data + self.yaw_val * self.yaw_sensitivity
                    pub_yaw.publish(self.yaw_f64)

                    self.drive_f64.data = self.drive_f64.data + self.drive_val * self.drive_sensitivity
                    pub_drive.publish(self.drive_f64)

                    self.strafe_f64.data = self.strafe_f64.data - self.strafe_val * self.strafe_sensitivity
                    pub_strafe.publish(self.strafe_f64)

                else:

                    self.yaw_f64 = Float64()
                    self.yaw_f64.data = self.yaw_val * self.thruster_power * self.twist_effort
                    pub_yaw.publish(self.yaw_f64)

                    self.drive_f64 = Float64()
                    self.drive_f64.data = self.drive_val * self.thruster_power
                    pub_drive.publish(self.drive_f64)

                    self.strafe_f64 = Float64()
                    self.strafe_f64.data = -1 * self.strafe_val * self.thruster_power
                    pub_strafe.publish(self.strafe_f64)

                depth_f64.data = depth_f64.data + self.depth_val * self.depth_sensitivity
                pub_depth.publish(depth_f64)

            pitch_f64.data = self.pitch_val*math.radians(45.0) # allow 15 deg pitch
            pub_pitch.publish(pitch_f64)
            #roll_f64.data = self.roll_val*math.radians(15.0) # allow 15 deg roll
            #pub_roll.publish(roll_f64)

            # 1.0 to -1.0, remap 0.65 to 0.0
            grip = self.gripper_val * 100.0

            outer_f64 = Float64(grip)
            inner_f64 = Float64(grip)
            gripper_outer_pub.publish(outer_f64)
            gripper_inner_pub.publish(inner_f64)

            # timer used to run in another thread
            if self.left_dropper_triggered:
                self.left_dropper_triggered = False
                self.left_dropper_avail = False
                rospy.Timer(rospy.Duration(0.1), self.actuate_left_dropper, oneshot=True)
            if self.right_dropper_triggered:
                self.right_dropper_triggered = False
                self.right_dropper_avail = False
                rospy.Timer(rospy.Duration(0.1), self.actuate_right_dropper, oneshot=True)
            if self.left_torpedo_triggered:
                self.left_torpedo_triggered = False
                self.left_torpedo_avail = False
                rospy.Timer(rospy.Duration(0.1), self.actuate_left_torpedo, oneshot=True)
            if self.right_torpedo_triggered:
                self.right_torpedo_triggered = False
                self.right_torpedo_avail = False
                rospy.Timer(rospy.Duration(0.1), self.actuate_right_torpedo, oneshot=True)

            rate.sleep()

if __name__ == '__main__':
    JOY_TELEOP = JoyTeleop()
    try:
        JOY_TELEOP.joy_teleop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joy Setpoint Teleop has died!")
