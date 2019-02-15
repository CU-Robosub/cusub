#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

STRAFE_AXES  = 0
DRIVE_AXES = 1
YAW_AXES = 2
DEPTH_AXES = 3

DEFAULT_DEPTH = 0

strafe_val  = 0.0
drive_val = 0.0
yaw_val = 0.0
depth_val = 0.0
pitch_val = 0.0
roll_val = 0.0

depth_f64 = None

TWIST_EFFORT    = 1.0
TRANLATE_EFFORT = 1.0

THRUSTER_POWER = 75.0

def joystick_state(data):

    global STRAFE_AXES
    global DRIVE_AXES
    global YAW_AXES
    global DEPTH_AXES
    global PITCH_AXES
    global ROLL_AXES

    global strafe_val
    global drive_val
    global yaw_val
    global depth_val
    global pitch_val
    global roll_val

    strafe_val  = data.axes[STRAFE_AXES]
    drive_val = data.axes[DRIVE_AXES]
    yaw_val = data.axes[YAW_AXES]
    depth_val = data.axes[DEPTH_AXES]
    pitch_val = data.axes[PITCH_AXES]
    roll_val = data.axes[ROLL_AXES]

def joy_teleop():

    global STRAFE_AXES
    global DRIVE_AXES
    global YAW_AXES
    global DEPTH_AXES
    global PITCH_AXES
    global ROLL_AXES

    global DEFAULT_DEPTH

    global strafe_val
    global drive_val
    global yaw_val
    global depth_val
    global pitch_val
    global roll_val

    pub_yaw = rospy.Publisher('/local_control/pid/yaw/control_effort', Float64, queue_size=10)
    pub_drive = rospy.Publisher('/local_control/pid/drive/control_effort', Float64, queue_size=10)
    pub_strafe = rospy.Publisher('/local_control/pid/strafe/control_effort', Float64, queue_size=10)

    pub_pitch = rospy.Publisher('/local_control/pid/pitch/setpoint', Float64, queue_size=10)
    pub_roll = rospy.Publisher('/local_control/pid/roll/setpoint', Float64, queue_size=10)

    pub_depth = rospy.Publisher('/local_control/pid/depth/setpoint', Float64, queue_size=10)

    rospy.init_node('joy_teleop', anonymous=True)

    STRAFE_AXES = rospy.get_param("~strafe_axes", STRAFE_AXES)
    DRIVE_AXES = rospy.get_param("~drive_axes", DRIVE_AXES)
    YAW_AXES = rospy.get_param("~yaw_axes", YAW_AXES)
    DEPTH_AXES = rospy.get_param("~depth_axes", DEPTH_AXES)
    PITCH_AXES = rospy.get_param("~pitch_axes", YAW_AXES)
    ROLL_AXES = rospy.get_param("~roll_axes", YAW_AXES)

    DEFAULT_DEPTH = rospy.get_param("~default_depth", DEFAULT_DEPTH)

    depth_f64 = Float64()
    depth_f64.data = DEFAULT_DEPTH

    pitch_f64 = Float64()
    pitch_f64.data = 0
    roll_f64 = Float64()
    roll_f64.data = 0

    rospy.Subscriber("/joy", Joy, joystick_state)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        yaw_f64 = Float64()
        yaw_f64.data = yaw_val*THRUSTER_POWER*TWIST_EFFORT
        pub_yaw.publish(yaw_f64)

        drive_f64 = Float64()
        drive_f64.data = drive_val*THRUSTER_POWER
        pub_drive.publish(drive_f64)

        strafe_f64 = Float64()
        strafe_f64.data = -1*strafe_val*THRUSTER_POWER
        pub_strafe.publish(strafe_f64)

        depth_f64.data = depth_f64.data - depth_val/10.0
        pub_depth.publish(depth_f64)

        #pitch_f64.data = pitch_val*math.radians(15.0) # allow 15 deg pitch
        #pub_pitch.publish(pitch_f64)
        #roll_f64.data = roll_val*math.radians(15.0) # allow 15 deg roll
        #pub_roll.publish(roll_f64)

        rate.sleep()

if __name__ == '__main__':
    try:
        joy_teleop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
