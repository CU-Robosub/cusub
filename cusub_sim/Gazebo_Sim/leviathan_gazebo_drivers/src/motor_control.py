#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64MultiArray, Float64
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

def motor_command_callback(data):

    global thrust

    thrust[0].data = (data.data[7] - 1500.0) / -3.0 # Right
    thrust[1].data = (data.data[6] - 1500.0) / -3.0 # Left

    thrust[2].data = (data.data[2] - 1420.0) / 1.5 # Front Left
    thrust[3].data = (data.data[1] - 1420.0) / 1.5 # Front Right
    thrust[4].data = (data.data[4] - 1420.0) / 1.5 # Back Left
    thrust[5].data = (data.data[3] - 1420.0) / 1.5 # Back Right

    thrust[6].data = (data.data[0] - 1500.0) / 3.0 # Front
    thrust[7].data = (data.data[5] - 1500.0) / 3.0 # Back

def motor_control():

    global thrust

    thrust = []
    for i in xrange(8):
        thrust.append(FloatStamped())
        thrust[i].data = 0

    rospy.init_node('gazebo_motor_control', anonymous=True)
    rospy.Subscriber('/drivers/pololu_control/command', Float64MultiArray, motor_command_callback)

    pub_thrust = []
    for i in xrange(8):
        pub_thrust.append(rospy.Publisher('/leviathan/thrusters/'+str(i)+'/input', FloatStamped, queue_size=1))

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        for i in xrange(8):
            pub_thrust[i].publish(thrust[i])

        rate.sleep()

if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
