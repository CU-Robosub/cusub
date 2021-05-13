#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import Empty, Bool, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped


class Depth_Sensor():
    '''
    This class reads data sent over serial from arduino and publishes a depth
    Zeros the depth sensor by sending information to arduino over seril
    '''

    def __init__(self):

	self.subname = rospy.get_param("~subname", "leviathan")

        self.zero_sub = rospy.Subscriber(
            "zero_command", Empty, self.zero_command, queue_size=1)

        self.pub_odom = rospy.Publisher(
            'depth_odom', PoseWithCovarianceStamped, queue_size=1)
        self.pub_map = rospy.Publisher(
            'depth_map', PoseWithCovarianceStamped, queue_size=1)

        self.pub_odom_data = PoseWithCovarianceStamped()
        self.pub_odom_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_odom_data.header.frame_id = self.subname+"/description/depth_odom_frame"

        self.pub_map_data = PoseWithCovarianceStamped()
        self.pub_map_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_map_data.header.frame_id = self.subname+"/description/depth_map_frame"

        self.ser = serial.Serial(
            '/dev/serial/by-id/usb-Adafruit_Industries_Trinket_M0-if00', '115200')

        self.kill_pub = rospy.Publisher('/kill_switch', Bool, queue_size=10)

    def zero_command(self, msg):
        pass
        # self.serial_zero() #send 0 to arduino, it will calibrate for zero

    def serial_zero(self):
        self.ser.write('a')

    def read_data(self):
        try:
            # parse

            # print("reading")
            a = self.ser.readline()

            #kswitch, adepth = a.split(",",1)

            pressure_mbar = float(a[1:])
            depth_m = -1*(pressure_mbar-797.11)*100/(1030*9.8)

            #kswitch = int(kswitch)

            """
            killed = Bool()

            if kswitch == 99:
              killed.data = True
            elif kswitch == 98:
              killed.data = False
	    else:
	      killed.data = False
            """
            # publish
            self.pub_odom_data.header.stamp = rospy.Time.now()
            self.pub_odom_data.header.seq += 1
            self.pub_odom_data.pose.pose.position.z = depth_m + 2.5

            self.pub_map_data.header.stamp = rospy.Time.now()
            self.pub_map_data.header.seq += 1
            self.pub_map_data.pose.pose.position.z = depth_m + 2.5

            self.pub_odom.publish(self.pub_odom_data)
            self.pub_map.publish(self.pub_map_data)

            # self.kill_pub.publish(killed)
        except Exception as e:
            print e

    def publish_depth(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():

            self.read_data()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('Depth_Pub')
    d = Depth_Sensor()
    try:
        d.publish_depth()
    except rospy.ROSInterruptException:
        pass
