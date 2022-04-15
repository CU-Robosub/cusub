#!/usr/bin/env python2
import rospy


from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


def callback(msg):
	x = []
	for i in range(10):
		x.append(msg.data)
	f = Float64MultiArray()
	f.data = x
	pub.publish(f)

pub = rospy.Publisher('/cusub_common/motor_controllers/pololu_control/command', Float64MultiArray, queue_size=10)
rospy.Subscriber('test', Float64, callback)

rospy.init_node("test")

rospy.spin()