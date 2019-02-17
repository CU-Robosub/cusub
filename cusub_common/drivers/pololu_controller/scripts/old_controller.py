#!/usr/bin/env python2

import termios, fcntl, sys, os
import contextlib
import rospy
import math
import time
# from pololu_controller.msg import MotorCommand
from std_msgs.msg import Float64MultiArray
import struct
import roslib
import binascii
roslib.load_manifest("rosparam")
import rosparam
import serial
#key stroke interrupt

# * Front: 0
# * front_right: 6
# * front_left: 2
# * back_right: 3
# * back_left: 5
# * back: 4
# * left: 9
# * right: 8

class Controller():
    def __init__(self):
        f = rospy.get_param("/pololu_controller/sub_yaml/")
        paramlist=rosparam.load_file(f,default_namespace="motors")
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)
        port = rospy.get_param("/pololu_controller/port_name/")
        baud = rospy.get_param("/pololu_controller/baud_rate/")
        self.port = serial.Serial(port, baud, timeout=0.5)
        self.port.flush()

        self.kill_all()
        self.sub = rospy.Subscriber('/drivers/pololu_control/command', Float64MultiArray, self.command_callback, queue_size=40)

    def command_callback(self,msg):
	# order: [front, frontright, frontleft, backright, backleft, back, left, right]
        arr = [msg.data[0],0,msg.data[2],msg.data[3],msg.data[5],msg.data[4],msg.data[1],0,msg.data[7],msg.data[6]]

        serialBytes = [
        0x9f,
        10,
        0
        ]
        for x in arr:
            pwn = x *4
            pwn = int(pwn)
            serialBytes.append(pwn & 0x7F)
            serialBytes.append((pwn >> 7) & 0x7F)

        self.port.write(serialBytes)
        binary_string = binascii.unhexlify(b'A1')
        self.port.write(binary_string)
        self.port.flush()
	print "Done"
        if(self.port.in_waiting > 0):
            s = self.port.read(2)
            out = struct.unpack('<H',s)
            print out[0]
    def kill_all(self):
        print("Killing all Motors")
        arr = [1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        serialBytes = [
        0x9f,
        10,
        0
        ]
        for x in arr:
            pwn = x *4
            pwn = int(pwn)
            serialBytes.append(pwn & 0x7F)
            serialBytes.append((pwn >> 7) & 0x7F)
        #print serialBytes
        self.port.write(serialBytes)
        binary_string = binascii.unhexlify(b'A1')
        self.port.write(binary_string)
        self.port.flush()
        if(self.port.in_waiting > 0):
            s = self.port.read(2)
            out = struct.unpack('<H',s)
                #print out[0]

m = Controller()

def signal_handler(singnal, frame):
    m.kill_all()
    print("\nprogram exited gracefully")
    sys.exit(0)
def main():
    rospy.init_node('Controller')

    rospy.spin()
    m.kill_all()


if __name__ == "__main__":
    main()
