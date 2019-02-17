#!/usr/bin/env python

import signal
import sys
import getpass
import sys
import telnetlib
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
HOST = "10.0.0.3"					#need to make this a static IP address
tn = telnetlib.Telnet(HOST, "9000")
def signal_handler(signal, frame):
	tn.write("\x03\r\n")
	print tn.read_until("OK")
	tn.write("MC\r\n")
	print tn.read_until("OK")
	tn.write("POWERDOWN\r\n")
	print tn.read_until("OK")
	tn.close()
	print("Shut down successful")
	sys.exit(0)

def DVL():

	pub_vel = rospy.Publisher('/drivers/dvl/vel',TwistWithCovarianceStamped, queue_size=100)
	pub_depth = rospy.Publisher('/drivers/dvl/depth', Float64MultiArray, queue_size=100)

	rospy.init_node('DVL', anonymous=True)
	rate = rospy.Rate(20) #1 Hz
	data_vel = TwistWithCovarianceStamped()
	data_depth = Float64MultiArray()


	print tn.read_until("Username:")
	tn.write("nortek\r\n")
	print tn.read_until("Password:")
	tn.write("nortek\r\n")
	print tn.read_until("Interface")
	tn.write("\x03\r\n")
	print tn.read_until("OK")
	tn.write("MC\r\n")
	print tn.read_until("OK")
	tn.write("START\r\n")
	print tn.read_until("OK")
	while not rospy.is_shutdown():
    # while (i < 10)
		tn.read_until("\r\n")
		x = tn.read_until("\r\n")
		# print x
		#velocity profile
		vx = float(x.split(",")[4].split("=")[1])
		vy = float(x.split(",")[5].split("=")[1])
		vz = float(x.split(",")[6].split("=")[1])

		#depth profile
		d1 = float(x.split(",")[8].split("=")[1])
		d2 = float(x.split(",")[9].split("=")[1])
		d3 = float(x.split(",")[10].split("=")[1])
		d4 = float(x.split(",")[11].split("=")[1].split("*")[0])
		data_vel.twist.covariance = [1e-9,   0,   0,0,0,0,
		0,   1e-9,  0,0,0,0,
		0,   0,   1e-9,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0]

		data_vel.header.stamp = rospy.Time.now()
		data_vel.header.frame_id = "base_dvl"
		data_vel.twist.twist.linear.x = -1*vx;
		data_vel.twist.twist.linear.y = vy;
		data_vel.twist.twist.linear.z = vz;


		#data_depth.header.stamp = rospy.Time.now()
		#data_depth.header.frame_id = "/drivers/dvl/depth"
		# data_depth.data = d1
		# data_depth.data = d2
		# data_depth.data = d3
		# data_depth.data = d4
		data_depth.data = [d1,d2,d2,d4]
		pub_vel.publish(data_vel)
		pub_depth.publish(data_depth)

		rate.sleep()
		signal.signal(signal.SIGINT, signal_handler)



if __name__ == '__main__':
	try:
		DVL()
	except rospy.ROSInterruptException:
		pass
