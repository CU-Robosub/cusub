#!/usr/bin/env python
"""
This module takes motor pwm commands and mapps them to thrust commands
for gazebo
"""

import rospy

from std_msgs.msg import Float64MultiArray
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class MotorControl(object):
    """This node takes motor pwm commands and mapps them to thrust commands
    for gazebo
    """

    thrust = []
    """list: List of FloatStamped containt motor thrust commands"""

    def __init__(self):
        pass

    def motor_command_callback(self, data):
        """Get motor pwms and generate thrust commands from them"""

        self.thrust[0].data = (data.data[7] - 1500.0) / -3.0 # Right
        self.thrust[1].data = (data.data[6] - 1500.0) / -3.0 # Left

        self.thrust[2].data = (data.data[2] - 1420.0) / 1.5 # Front Left
        self.thrust[3].data = (data.data[1] - 1420.0) / 1.5 # Front Right
        self.thrust[4].data = (data.data[4] - 1420.0) / 1.5 # Back Left
        self.thrust[5].data = (data.data[3] - 1420.0) / 1.5 # Back Right

        self.thrust[6].data = (data.data[0] - 1500.0) / 3.0 # Front
        self.thrust[7].data = (data.data[5] - 1500.0) / 3.0 # Back

    def motor_control(self):
        """Main loop manages updating motor thrusts"""

        for i in xrange(8):
            self.thrust.append(FloatStamped())
            self.thrust[i].data = 0

        rospy.init_node('gazebo_motor_control', anonymous=True)

        rospy.Subscriber('drivers/pololu_control/command',
                         Float64MultiArray, self.motor_command_callback)

        pub_thrust = []
        for i in xrange(8):
            pub_thrust.append(rospy.Publisher('thrusters/' \
                                              + str(i) + '/input',
                                              FloatStamped, queue_size=1))

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            for i in xrange(8):
                pub_thrust[i].publish(self.thrust[i])

            rate.sleep()

if __name__ == '__main__':
    MOTOR_CONTROL = MotorControl()
    try:
        MOTOR_CONTROL.motor_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Motor control died!")
