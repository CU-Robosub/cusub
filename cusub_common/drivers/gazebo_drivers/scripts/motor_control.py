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

        # Flip motors based on motor flip array to match reality
        flip_motor_array = rospy.get_param('flip_motor_array')

        self.thrust[0].data = (data.data[0] - 1500.0) /  3.0 * flip_motor_array[0]
        self.thrust[1].data = (data.data[1] - 1500.0) /  3.0 * flip_motor_array[1]
        self.thrust[2].data = (data.data[2] - 1500.0) /  3.0 * flip_motor_array[2]
        self.thrust[3].data = (data.data[3] - 1500.0) /  3.0 * flip_motor_array[3]
        self.thrust[4].data = (data.data[4] - 1500.0) /  3.0 * flip_motor_array[4]
        self.thrust[5].data = (data.data[5] - 1500.0) /  3.0 * flip_motor_array[5]
        self.thrust[6].data = (data.data[6] - 1500.0) /  3.0 * flip_motor_array[6]
        self.thrust[7].data = (data.data[7] - 1500.0) /  3.0 * flip_motor_array[7]

    def motor_control(self):
        """Main loop manages updating motor thrusts"""

        for i in xrange(8):
            self.thrust.append(FloatStamped())
            self.thrust[i].data = 0

        rospy.init_node('gazebo_motor_control', anonymous=True)

        rospy.Subscriber('cusub_common/motor_controllers/pololu_control/command',
                         Float64MultiArray, self.motor_command_callback)

        pub_thrust = []

        for i in xrange(8):

            pub_thrust.append(rospy.Publisher('description/thrusters/' \
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
