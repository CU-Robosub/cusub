#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import rospy
import time

from std_msgs.msg import Empty, Bool, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped


class Depth_Sensor():
    '''
    This class reads data sent over serial from arduino and publishes a depth
    Zeros the depth sensor by sending information to arduino over seril
    '''

    def __init__(self):

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
        self.pub_odom_data.header.frame_id = "leviathan/description/depth_odom_frame"

        self.pub_map_data = PoseWithCovarianceStamped()
        self.pub_map_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_map_data.header.frame_id = "leviathan/description/depth_map_frame"

        self.ser = serial.Serial(
            '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN05ZN8B-if00-port0',
            115200)

    def zero_command(self, msg):
        pass

    def publish_depth(self):

        # Ask for calibration coeffcients
        #self.ser.write("\xC0")
        #time.sleep(1)
        #self.ser.write("\x33")
        #time.sleep(1)
        #self.ser.write("\xC0")

        data = b''
        C = [0, 29689, 30037, 17723, 18869, 29324, 26604] #[0]

        while not rospy.is_shutdown():

            data_byte = self.ser.read(1)

            if ord(data_byte[0]) == 0xC0:

                if len(data) > 0:

                    data = data.replace('\xDB\xDC', '\xC0').replace('\xDB\xDD', '\xDB')

                    print("%02X" % ord(data[0]))

                    if ord(data[0]) == 0xEE: # POWER_CURRENT

                        p1, p2, p3, p4, p5, p6 = struct.unpack("<HHHHHH", data[1:])
                        bat_voltage = (14.1*1.2*p6/(2**14))

                        print("BAT: %.2fv" % bat_voltage)

                    elif ord(data[0]) == 0x00 and len(data) > 1:

                        pass

                        #C1, C2, C3, C4, C5, C6 = struct.unpack(">HHHHHH", data[1:])
                        #C += [C1, C2, C3, C4, C5, C6]

                        #print(C)

                    elif ord(data[0]) == 0xCC:

                        if len(C) > 1:

                            TEMP = 0
                            P = 0

                            # Read Pressure Result
                            D1 = struct.unpack(">I", "\x00"+data[1:4])[0]

                            # Read Pressure Result
                            D2 = struct.unpack(">I", "\x00"+data[4:7])[0]

                            dT = D2 - C[5]*256.0
                            TEMP = 2000.0 + dT*C[6]/8388608.0

                            OFF = C[2]*65536.0 + (C[4]*dT)/128.0
                            SENS = C[1]*32768.0 + (C[3]*dT)/256.0
                            P = (D1*SENS/2097152.0 - OFF)/8192.0

                            temperature_c = TEMP / 100.0

                            pressure_mbar = P / 10.0
                            depth_m = -1*(pressure_mbar-797.11)*100/(1030*9.8)

                            #print((u"Temperature: %.2f °C" % temperature_c).encode("utf-8"))
                            #print("Pressure: %.2f mBar" % pressure_mbar)
                            print("Depth: %.2f m" % depth_m)

                            self.pub_odom_data.header.stamp = rospy.Time.now()
                            self.pub_odom_data.header.seq += 1
                            self.pub_odom_data.pose.pose.position.z = depth_m

                            self.pub_map_data.header.stamp = rospy.Time.now()
                            self.pub_map_data.header.seq += 1
                            self.pub_map_data.pose.pose.position.z = depth_m

                            self.pub_odom.publish(self.pub_odom_data)
                            self.pub_map.publish(self.pub_map_data)


                data = b''

            else:
                data += data_byte

if __name__ == "__main__":

    rospy.init_node('Depth_Pub')

    d = Depth_Sensor()
    try:

        d.publish_depth()

    except rospy.ROSInterruptException:
        rospy.logerror("Depth_Pub Died!")
