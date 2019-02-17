#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

class Depth_Sensor():
    '''
    This class reads data sent over serial from arduino and publishes a depth
    Zeros the depth sensor by sending information to arduino over seril
    '''
    def __init__(self):
        self.zero_sub = rospy.Subscriber("zero_command", Empty, self.zero_command, queue_size =1)
        self.pub_Pose = rospy.Publisher('depth', PoseWithCovarianceStamped,queue_size=1)
        self.pub_Pose_data = PoseWithCovarianceStamped()
	self.pub_Pose_data.pose.covariance = [0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0.01 , 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0]
        self.pub_Pose_data.header.frame_id = "depth_frame"
        self.ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_95335343136351104201-if00', '115200')


    def zero_command(self, msg):
        self.serial_zero() #send 0 to arduino, it will calibrate for zero

    def serial_zero(self):
        self.ser.write('a')

    def read_data(self):
        try:
            a = self.ser.readline()
            serial_input = float(a)
            # publish
            self.pub_Pose_data.header.stamp = rospy.Time.now()
            self.pub_Pose_data.header.seq += 1
            self.pub_Pose_data.pose.pose.position.z = serial_input
            self.pub_Pose.publish(self.pub_Pose_data)
        except Exception as e:
	    print e
            pass

    def publish_depth(self):
        r = rospy.Rate(115200)
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
