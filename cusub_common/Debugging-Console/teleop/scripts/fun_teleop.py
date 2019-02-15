#!/usr/bin/env python
# import termios, fcntl, sys, os
# import contextlib
import rospy
# import math
# import time
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
# from std_msgs.msg import Bool
# from pololu_controller.msg import MotorCommand
# from sensor_msgs.msg import Imu
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

        self.raw_motor_pub = rospy.Publisher('/drivers/pololu_control/command', Float64MultiArray, queue_size=1)

        self.zero_array_data = Float64MultiArray()
        self.zero_array_data.data = [1500,1500,1500,1500,1500,1500,1500,1500]

        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.depth = 0
        self.drive = 0
        self.strafe = 0

        self.drive_increment = 0.5
        self.yaw_increment = 10


    def pub_yaw(self, increase=True):
        if increase:
            self.yaw+=self.yaw_increment
        else:
            self.yaw-=self.yaw_increment

        self.yaw_setpoint_pub_data=self.yaw

        self.yaw_setpoint_pub.publish(self.yaw_setpoint_pub_data)

    # def pub_roll(self):
    #     self.roll = angle
    #     self.roll_setpoint_pub_data = angle*(math.pi/180)
    #     self.roll_setpoint_pub.publish(self.roll_setpoint_pub_data)
    #
    # def pub_pitch(self):
    #     self.pitch = angle
    #     self.pitch_setpoint_pub_data = angle*(math.pi/180)
    #     self.pitch_setpoint_pub.publish(self.pitch_setpoint_pub_data)
    #
    def pub_depth(self, increase=True):
        if increase:
            self.depth+=self.drive_increment
        else:
            self.depth-=self.drive_increment

        self.depth_setpoint_pub_data = self.depth

        self.depth_setpoint_pub.publish(self.depth_setpoint_pub_data)

    def pub_drive(self, increase=True):
        if increase:
            self.drive+=self.drive_increment
        else:
            self.drive-=self.drive_increment

        self.drive_setpoint_pub_data = self.drive

        self.drive_setpoint_pub.publish(self.drive_setpoint_pub_data)

    def pub_strafe(self, increase=True):
        if increase:
            self.strafe+=self.drive_increment
        else:
            self.strafe-=self.drive_increment

        self.strafe_setpoint_pub_data = self.strafe

        self.strafe_setpoint_pub.publish(self.strafe_setpoint_pub_data)

    def kill_motors(self):
        self.raw_motor_pub.publish(self.zero_array_data)

    def start_driving(self):
        while not rospy.is_shutdown():
            print("type kill to kill")
            c = raw_input()
            c = str(c)
            if(c == 'w'):
                self.pub_drive()
            elif(c == 's'):
                self.pub_drive(increase=False)
            elif(c == 'a'):
                self.pub_strafe(increase=False)
            elif(c == 'd'):
                self.pub_strafe()
            elif(c == 'q'):
                self.pub_yaw(increase=False)
            elif(c == 'e'):
                self.pub_yaw()
            elif(c == 'z'):
                self.pub_depth()
            elif(c == 'x'):
                self.pub_depth(increase=False)
            elif(c == 'kill'):
                self.kill_motors()
                print("exitted gracefully")
                return


def main():
    rospy.init_node('Motor_Controller')
    m = Motor_Controller()

    try:
        m.start_driving()
    except rospy.ROSInterruptException:
        m.kill_motors()
        print("exitted gracefully")
        return


if __name__ == "__main__":
    main()
