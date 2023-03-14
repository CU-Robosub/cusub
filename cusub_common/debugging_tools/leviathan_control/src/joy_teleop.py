#!/usr/bin/env python
"""
Allows joystick teleoperation of submarine
"""

import math

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from actuator.srv import ActivateActuator

class JoyTeleop(object):

    SETPOINT_MODE = 'setpoint'
    DIRECT_MODE = 'direct'
    QUAD_DIRECT_MODE = 'quad_direct'
    QUAD_MODE = 'quad'

    strafe_axes  = 0
    drive_axes = 1
    yaw_axes = 2
    depth_axes = 5
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

    thruster_power = 140.0

    yaw_sensitivity = 0.01
    drive_sensitivity = 0.02
    strafe_sensitivity = 0.02
    depth_sensitivity = 0.005

    mode_yaw_sensitivity = 1
    mode_drive_sensitivity = 1
    mode_strafe_sensitivity = 1

    dropper_triggered = False
    left_torpedo_triggered = False
    right_torpedo_triggered = False

    gripper_state = False
    servo_state = 0.0
    dropper_avail = True
    left_torpedo_avail = True
    right_torpedo_avail = True

    current_drive_updated = False
    current_strafe_updated = False

    def joystick_state(self, data):
        """Gets joystick data to figure out what to have the sub do
        """

        if data.buttons[0]:
            self.gripper_state = True
        else:
            self.gripper_state = False
        #if data.buttons[0] and self.dropper_avail:
        #    self.dropper_triggered = True
        self.servo_state = (1.0 + data.axes[3])/2.0
        if data.buttons[4] and self.left_torpedo_avail:
            self.left_torpedo_triggered = True
        if data.buttons[5] and self.right_torpedo_avail:
            self.right_torpedo_triggered = True

        self.strafe_val  = data.axes[self.strafe_axes] * self.mode_strafe_sensitivity
        # self.strafe_val  = 0
        self.drive_val = data.axes[self.drive_axes] * self.mode_drive_sensitivity
        self.yaw_val = data.axes[self.yaw_axes] * self.mode_yaw_sensitivity
        # self.depth_val = - data.buttons[1] + data.buttons[2]
        self.depth_val = -1 * (-data.axes[self.depth_axes] + 4 * (data.buttons[2] - data.buttons[4]))
        #self.pitch_val = data.axes[self.pitch_axes]
        self.pitch_val = 0
        #self.roll_val = data.axes[self.roll_axes]
        self.roll_val = 0
        #self.gripper_val = data.axes[self.gripper_axes]
        self.gripper_val = data.buttons[0]

        #fine-ctrl mode on
        if data.buttons[10] == 1:
            print("slow mode activated")
            self.mode_yaw_sensitivity = .25
            self.mode_drive_sensitivity = .5
            self.mode_strafe_sensitivity = 1
            
        
        #fine-ctrl mode off
        if data.buttons[11] == 1:
            print("slow mode deactivated")
            self.mode_yaw_sensitivity = 1
            self.mode_drive_sensitivity = 1
            self.mode_strafe_sensitivity = 1

    def driveStateCallback(self, data):
        if not self.current_drive_updated:
            self.drive_f64.data = data.data
            self.current_drive_updated = True

    def strafeStateCallback(self, data):
        if not self.current_strafe_updated:
            self.strafe_f64.data = data.data
            self.current_strafe_updated = True

    @staticmethod
    def actuate(num, time):
        activate_actuator = rospy.ServiceProxy('activateActuator', ActivateActuator)
        activate_actuator(num, time)

    def actuate_dropper(self, _):
        self.actuate(1, 500)
        self.dropper_avail = True

    def actuate_left_torpedo(self, _):
        self.actuate(2, 500)
        self.left_torpedo_avail = True

    def actuate_right_torpedo(self, _):
        self.actuate(3, 500)
        self.right_torpedo_avail = True

    def joy_teleop(self):

        rospy.init_node('joy_teleop', anonymous=True)

        mode = rospy.get_param('~setpoint', 0)

        self.drive_f64 = Float64()
        self.drive_f64.data = 0.0
        self.strafe_f64 = Float64()
        self.strafe_f64.data = 0.0

        self.gripper_bool = Bool()
        self.gripper_bool.data = False

        if mode == JoyTeleop.SETPOINT_MODE:

            pub_yaw = rospy.Publisher('motor_controllers/pid/yaw/setpoint',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('motor_controllers/pid/drive/setpoint',
                                        Float64, queue_size=10)
            pub_strafe = rospy.Publisher('motor_controllers/pid/strafe/setpoint',
                                         Float64, queue_size=10)

            # get current accumulated strafe and drive to use for adjustments
            rospy.Subscriber("motor_controllers/pid/drive/state",
                             Float64, self.driveStateCallback)
            rospy.Subscriber("motor_controllers/pid/strafe/state",
                             Float64, self.strafeStateCallback)

            pub_pitch = rospy.Publisher('motor_controllers/pid/pitch/setpoint',
                                        Float64, queue_size=10)
            pub_roll = rospy.Publisher('motor_controllers/pid/roll/setpoint',
                                       Float64, queue_size=10)

        elif mode == JoyTeleop.QUAD_MODE:

            pub_yaw = rospy.Publisher('motor_controllers/pid/yaw/setpoint',
                                      Float64, queue_size=10)

            pub_pitch = rospy.Publisher('motor_controllers/pid/pitch/control_effort',
                                        Float64, queue_size=10)

            pub_roll = rospy.Publisher('motor_controllers/pid/roll/control_effort',
                                       Float64, queue_size=10)

            pub_drive = rospy.Publisher('motor_controllers/mux/drive/control_effort',
                                         Float64, queue_size=10)

            pub_strafe = rospy.Publisher('motor_controllers/mux/strafe/control_effort',
                                         Float64, queue_size=10)

        elif mode == JoyTeleop.DIRECT_MODE: 

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

        elif mode == JoyTeleop.QUAD_DIRECT_MODE: 

            pub_yaw = rospy.Publisher('motor_controllers/mux/yaw/control_effort',
                                      Float64, queue_size=10)
            pub_drive = rospy.Publisher('motor_controllers/mux/drive/control_effort',
                                         Float64, queue_size=10)
            pub_strafe = rospy.Publisher('motor_controllers/mux/strafe/control_effort',
                                         Float64, queue_size=10)

            pub_pitch = rospy.Publisher('motor_controllers/pid/pitch/control_effort',
                                        Float64, queue_size=10)
            pub_roll = rospy.Publisher('motor_controllers/pid/roll/control_effort',
                                       Float64, queue_size=10)

        pub_depth = rospy.Publisher('motor_controllers/pid/depth/setpoint',
                                    Float64, queue_size=10)


        self.gripper_pub = rospy.Publisher('motor_controllers/gripper_state', Bool, queue_size=1)
        self.servo_pub = rospy.Publisher('motor_controllers/servo_state', Float64, queue_size=1)

        gripper_outer_pub = rospy.Publisher('/leviathan/description/outer_controller/command', Float64, queue_size=1)
        gripper_inner_pub = rospy.Publisher('/leviathan/description/inner_controller/command', Float64, queue_size=1)

        self.strafe_axes = rospy.get_param("~strafe_axes", self.strafe_axes)
        self.drive_axes = rospy.get_param("~drive_axes", self.drive_axes)
        self.yaw_axes = rospy.get_param("~yaw_axes", self.yaw_axes)
        self.depth_axes = rospy.get_param("~depth_axes", self.depth_axes)
        self.pitch_axes = rospy.get_param("~pitch_axes", self.pitch_axes)
        self.roll_axes = rospy.get_param("~roll_axes", self.roll_axes)

        self.default_depth = rospy.get_param("~default_depth", self.default_depth)

        depth_f64 = Float64()
        depth_f64.data = self.default_depth

        yaw_f64 = Float64()
        yaw_f64.data = 0.0

        pitch_f64 = Float64()
        pitch_f64.data = 0
        roll_f64 = Float64()
        roll_f64.data = 0

        rospy.Subscriber("joy", Joy, self.joystick_state)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            if mode == JoyTeleop.SETPOINT_MODE:

                yaw_f64.data = yaw_f64.data + self.yaw_val * self.yaw_sensitivity
                pub_yaw.publish(yaw_f64)

                self.drive_f64.data = self.drive_f64.data + self.drive_val * self.drive_sensitivity
                pub_drive.publish(self.drive_f64)

                self.strafe_f64.data = self.strafe_f64.data - self.strafe_val * self.strafe_sensitivity
                pub_strafe.publish(self.strafe_f64)

                pitch_f64.data = self.pitch_val*math.radians(45.0) # allow 15 deg pitch
                pub_pitch.publish(pitch_f64)
                #roll_f64.data = self.roll_val*math.radians(15.0) # allow 15 deg roll
                #pub_roll.publish(roll_f64)

            elif mode == JoyTeleop.QUAD_MODE:

                yaw_f64.data = yaw_f64.data + self.yaw_val * self.yaw_sensitivity
                pub_yaw.publish(yaw_f64)

                self.drive_f64 = Float64()
                self.drive_f64.data = self.drive_val * self.thruster_power
                pub_drive.publish(self.drive_f64)

                self.strafe_f64 = Float64()
                self.strafe_f64.data = -1 * self.strafe_val * self.thruster_power
                pub_strafe.publish(self.strafe_f64)

                roll_f64 = Float64()
                roll_f64.data = self.roll_val * self.thruster_power * -0.5
                pub_roll.publish(roll_f64)

            elif mode == JoyTeleop.DIRECT_MODE: 

                yaw_f64 = Float64()
                yaw_f64.data = self.yaw_val * self.thruster_power * self.twist_effort
                pub_yaw.publish(yaw_f64)

                self.drive_f64 = Float64()
                self.drive_f64.data = self.drive_val * self.thruster_power
                pub_drive.publish(self.drive_f64)

                self.strafe_f64 = Float64()
                self.strafe_f64.data = -1 * self.strafe_val * self.thruster_power
                pub_strafe.publish(self.strafe_f64)

                pitch_f64.data = self.pitch_val*math.radians(45.0) # allow 15 deg pitch
                pub_pitch.publish(pitch_f64)
                #roll_f64.data = self.roll_val*math.radians(15.0) # allow 15 deg roll
                #pub_roll.publish(roll_f64)

            elif mode == JoyTeleop.QUAD_DIRECT_MODE: 

                yaw_f64 = Float64()
                yaw_f64.data = self.yaw_val * self.thruster_power * self.twist_effort
                pub_yaw.publish(yaw_f64)

                self.drive_f64 = Float64()
                self.drive_f64.data = self.drive_val * self.thruster_power
                pub_drive.publish(self.drive_f64)

                self.strafe_f64 = Float64()
                self.strafe_f64.data = -1 * self.strafe_val * self.thruster_power
                pub_strafe.publish(self.strafe_f64)

                pitch_f64 = Float64()
                pitch_f64.data = self.pitch_val * self.thruster_power * 0.5
                pub_pitch.publish(pitch_f64)

                roll_f64 = Float64()
                roll_f64.data = self.roll_val * self.thruster_power * -0.5
                pub_roll.publish(roll_f64)

            depth_f64.data = depth_f64.data + self.depth_val * self.depth_sensitivity
            pub_depth.publish(depth_f64)

            # 1.0 to -1.0, remap 0.65 to 0.0
            grip = self.gripper_val * 100.0

            gripper_bool = self.gripper_state
            self.gripper_pub.publish(gripper_bool)

            servo_f64 = Float64()
            servo_f64.data = self.servo_state
            self.servo_pub.publish(servo_f64)

            outer_f64 = Float64(grip)
            inner_f64 = Float64(grip)
            gripper_outer_pub.publish(outer_f64)
            gripper_inner_pub.publish(inner_f64)

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
