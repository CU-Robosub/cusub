#!/usr/bin/env python

import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sensor_msgs.msg import Joy

class TorpedoJoy(object):

    def joystick_state(self, data):

        self.drive_val = data.axes[4]
        self.yaw_val = data.axes[0]
        self.pitch_val = data.axes[1]

    def run(self):

        rospy.init_node('torpedo_joy', anonymous=True)

        self.drive_val = 0.0
        self.drive_f64 = FloatStamped()
        self.drive_f64.data = 0.0
        self.yaw_val = 0.0
        self.yaw_f64 = FloatStamped()
        self.yaw_f64.data = 0.0
        self.pitch_val = 0.0
        self.pitch_f64 = FloatStamped()
        self.pitch_f64.data = 0.0

        pub_yaw = rospy.Publisher('/baby_yoda/description/fins/0/input',
                                    FloatStamped, queue_size=10)
        pub_pitch = rospy.Publisher('/baby_yoda/description/fins/1/input',
                                    FloatStamped, queue_size=10)
        pub_drive = rospy.Publisher('/baby_yoda/description/thrusters/0/input',
                                    FloatStamped, queue_size=10)

        rospy.Subscriber("joy", Joy, self.joystick_state)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.yaw_f64.data = self.yaw_val * -0.7
            pub_yaw.publish(self.yaw_f64)

            self.pitch_f64.data = self.pitch_val * 0.7
            pub_pitch.publish(self.pitch_f64)

            self.drive_f64.data = self.drive_val * 50.0
            pub_drive.publish(self.drive_f64)

            rate.sleep()

if __name__ == '__main__':
    torpedo_joy = TorpedoJoy()
    try:
        torpedo_joy.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Torpedo Joy has died!")
