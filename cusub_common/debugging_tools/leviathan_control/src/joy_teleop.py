#!/usr/bin/env python
"""
Allows joystick teleoperation of submarine
"""

import math

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

from actuator.srv import ActivateActuator

class JoyTeleop(object):

    strafe_axes  = 0
    drive_axes = 1
    yaw_axes = 2
    depth_axes = 3
    pitch_axes = 2
    roll_axes = 2

    default_depth = 0

    strafe_val  = 0.0
    drive_val = 0.0
    yaw_val = 0.0
    depth_val = 0.0
    pitch_val = 0.0
    roll_val = 0.0

    depth_f64 = None

    twist_effort    = 1.0
    translate_effort = 1.0

    thruster_power = 75.0

    yaw_sensitivity = 0.02
    drive_sensitivity = 0.02
    strafe_sensitivity = 0.02
    depth_sensitivity = 0.02

    dropper_triggered = False
    left_torpedo_triggered = False
    right_torpedo_triggered = False

    dropper_avail = True
    left_torpedo_avail = True
    right_torpedo_avail = True

    def joystick_state(self, data):
        """Gets joystick data to figure out what to have the sub do
        """

        if data.buttons[0] and self.dropper_avail:
            self.dropper_triggered = True
        if data.buttons[4] and self.left_torpedo_avail:
            self.left_torpedo_triggered = True
        if data.buttons[5] and self.right_torpedo_avail:
            self.right_torpedo_triggered = True

        self.strafe_val  = data.axes[self.strafe_axes]
        self.drive_val = data.axes[self.drive_axes]
        self.yaw_val = data.axes[self.yaw_axes]
        self.depth_val = data.axes[self.depth_axes]
        self.pitch_val = data.axes[self.pitch_axes]
        self.roll_val = data.axes[self.roll_axes]

    @staticmethod
    def actuate(num, time):
        activate_actuator = rospy.ServiceProxy('activateActuator', ActivateActuator)
        activate_actuator(num, time)

    def actuate_dropper(self, _):
        self.actuate(1, 100)
        self.dropper_avail = True

    def actuate_left_torpedo(self, _):
        self.actuate(2, 100)
        self.left_torpedo_avail = True

    def actuate_right_torpedo(self, _):
        self.actuate(3, 100)
        self.right_torpedo_avail = True

    def joy_teleop(self):

        rospy.init_node('joy_teleop', anonymous=True)

        setpoint = rospy.get_param('~setpoint', True)

        if setpoint:

            pub_yaw = rospy.Publisher('cusub_common/motor_controllers/pid/yaw/setpoint',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('cusub_common/motor_controllers/pid/drive/setpoint',
                                        Float64, queue_size=10)
            pub_strafe = rospy.Publisher('cusub_common/motor_controllers/pid/strafe/setpoint',
                                         Float64, queue_size=10)

        else: 

            pub_yaw = rospy.Publisher('cusub_common/motor_controllers/mux/yaw/control_effort',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('cusub_common/motor_controllers/mux/drive/control_effort',
                                         Float64, queue_size=10)
            pub_strafe = rospy.Publisher('cusub_common/motor_controllers/mux/strafe/control_effort',
                                         Float64, queue_size=10)

        pub_pitch = rospy.Publisher('cusub_common/motor_controllers/pid/pitch/setpoint',
                                    Float64, queue_size=10)
        pub_roll = rospy.Publisher('cusub_common/motor_controllers/pid/roll/setpoint',
                                   Float64, queue_size=10)

        pub_depth = rospy.Publisher('cusub_common/motor_controllers/pid/depth/setpoint',
                                    Float64, queue_size=10)

        self.strafe_axes = rospy.get_param("~strafe_axes", self.strafe_axes)
        self.drive_axes = rospy.get_param("~drive_axes", self.drive_axes)
        self.yaw_axes = rospy.get_param("~yaw_axes", self.yaw_axes)
        self.depth_axes = rospy.get_param("~depth_axes", self.depth_axes)
        self.pitch_axes = rospy.get_param("~pitch_axes", self.pitch_axes)
        self.roll_axes = rospy.get_param("~roll_axes", self.roll_axes)

        self.default_depth = rospy.get_param("~default_depth", self.default_depth)

        depth_f64 = Float64()
        depth_f64.data = self.default_depth

        drive_f64 = Float64()
        drive_f64.data = 0.0
        strafe_f64 = Float64()
        strafe_f64.data = 0.0
        yaw_f64 = Float64()
        yaw_f64.data = 0.0

        pitch_f64 = Float64()
        pitch_f64.data = 0
        roll_f64 = Float64()
        roll_f64.data = 0

        rospy.Subscriber("joy", Joy, self.joystick_state)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            if setpoint:

                yaw_f64.data = yaw_f64.data + self.yaw_val * self.yaw_sensitivity
                pub_yaw.publish(yaw_f64)

                drive_f64.data = drive_f64.data + self.drive_val * self.drive_sensitivity
                pub_drive.publish(drive_f64)

                strafe_f64.data = strafe_f64.data - self.strafe_val * self.strafe_sensitivity
                pub_strafe.publish(strafe_f64)

            else:

                yaw_f64 = Float64()
                yaw_f64.data = self.yaw_val * self.thruster_power * self.twist_effort
                pub_yaw.publish(yaw_f64)

                drive_f64 = Float64()
                drive_f64.data = self.drive_val * self.thruster_power
                pub_drive.publish(drive_f64)

                strafe_f64 = Float64()
                strafe_f64.data = -1 * self.strafe_val * self.thruster_power
                pub_strafe.publish(strafe_f64)

            depth_f64.data = depth_f64.data + self.depth_val * self.depth_sensitivity
            pub_depth.publish(depth_f64)

            pitch_f64.data = self.pitch_val*math.radians(15.0) # allow 15 deg pitch
            pub_pitch.publish(pitch_f64)
            roll_f64.data = self.roll_val*math.radians(15.0) # allow 15 deg roll
            pub_roll.publish(roll_f64)

            # timer used to run in another thread
            if self.dropper_triggered:
                self.dropper_triggered = False
                self.dropper_avail = False
                rospy.Timer(rospy.Duration(0.1), self.actuate_dropper, oneshot=True)
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
