#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from pololu_controller.msg import MotorCommand
from sensor_msgs.msg import Imu
#front left back_left backwards
class Motor_Controller():
    def __init__(self):

        self.yaw_setpoint_pub = rospy.Publisher('/local_control/pid/yaw/setpoint',Float64,queue_size=1)
        self.yaw_setpoint_pub_data = Float64()
        self.roll_setpoint_pub = rospy.Publisher('/local_control/pid/roll/setpoint',Float64,queue_size=1)
        self.roll_setpoint_pub_data = Float64()
        self.pitch_setpoint_pub = rospy.Publisher('/local_control/pid/pitch/setpoint',Float64,queue_size=1)
        self.pitch_setpoint_pub_data = Float64()
        self.depth_setpoint_pub = rospy.Publisher('/local_control/pid/depth/setpoint',Float64,queue_size=1)
        self.depth_setpoint_pub_data = Float64()
        self.drive_setpoint_pub = rospy.Publisher('/local_control/pid/drive/setpoint',Float64,queue_size=1)
        self.drive_setpoint_pub_data = Float64()
        self.strafe_setpoint_pub = rospy.Publisher('/local_control/pid/strafe/setpoint',Float64,queue_size=1)
        self.strafe_setpoint_pub_data = Float64()

        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.depth = -1
        self.drive = 0
        self.strafe = 0


    def pub_yaw(self,angle):
        self.yaw = angle
        self.yaw_setpoint_pub_data = angle*(math.pi/180)
        self.yaw_setpoint_pub.publish(self.yaw_setpoint_pub_data)
    def pub_roll(self,angle):
        self.roll = angle
        self.roll_setpoint_pub_data = angle*(math.pi/180)
        self.roll_setpoint_pub.publish(self.roll_setpoint_pub_data)
    def pub_pitch(self,angle):
        self.pitch = angle
        self.pitch_setpoint_pub_data = angle*(math.pi/180)
        self.pitch_setpoint_pub.publish(self.pitch_setpoint_pub_data)
    def pub_depth(self,pos):
        self.depth = pos
        self.depth_setpoint_pub_data = pos
        self.depth_setpoint_pub.publish(self.depth_setpoint_pub_data)
    def pub_drive(self,pos):
        self.drive = pos
        self.drive_setpoint_pub_data = pos
        self.drive_setpoint_pub.publish(self.drive_setpoint_pub_data)
    def pub_strafe(self,pos):
        self.strafe = pos
        self.strafe_setpoint_pub_data = pos
        self.strafe_setpoint_pub.publish(self.strafe_setpoint_pub_data)



def main():
    rospy.init_node('Motor_Controller')
    m = Motor_Controller()

    forward = math.pi
    backward = -math.pi
    stop = 0
    while not rospy.is_shutdown():
        c = raw_input("Enter DOF to change setpoint:\n1 - Yaw : "+str(m.yaw)+"\n2 - Roll : "+str(m.roll)+"\n3 - pitch : "+str(m.pitch)+"\n4 - depth : "+str(m.depth)+"\n5 - drive : "+str(m.drive)+"\n6 - strafe : "+str(m.strafe)+"\n")
        if(c == '1'):
            a = input("Enter an angle:")
            m.pub_yaw(float(a))
        elif(c == '2'):
            a = input("Enter an angle:")
            m.pub_roll(float(a))
        elif(c == '3'):
            a = input("Enter an angle:")
            m.pub_pitch(float(a))
        elif(c == '4'):
            a = input("Enter an pos:")
            m.pub_depth(float(a))
        elif(c == '5'):
            a = input("Enter an pos:")
            m.pub_drive(float(a))
        elif(c == '6'):
            a = input("Enter an pos:")
            m.pub_strafe(float(a))



if __name__ == "__main__":
    main()
