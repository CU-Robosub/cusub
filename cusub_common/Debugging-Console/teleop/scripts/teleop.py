#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import time
# from pololu_controller.msg import MotorCommand
from std_msgs.msg import Float64MultiArray
import sys, signal

# * Front: 0
# * front_right: 6
# * front_left: 2
# * back_right: 3
# * back_left: 5
# * back: 4
# * left: 9
# * right: 8
#motor to keyboad mappings
motor_kybrd = {'w': '0', 'e': '1', 'q': '2',
          'x': '5', 'c': '3', 'z': '4',
          'a': '6', 'd': '7'}

#key stroke interrupt
def signal_handler(singnal, frame):
    print("\nprogram exited gracefully")
    sys.exit(0)

#TODO remove class? if < 5 functions yes
class Motor_Controller():
    def __init__(self):
        #ONE Publisher for sending commands to the pololu
        self.motor_pub = rospy.Publisher('/drivers/pololu_control/command', Float64MultiArray, queue_size=1)
        self.motor_cmd = Float64MultiArray()
        # self.motor_cmd.speed = 0
        # self.motor_cmd.acceleration = 0
        self.arr = [1500,1500,1500,1500,1500,1500,1500,1500]


    def single_motor(self, motor, intensity):
        print motor
        print intensity

        i = int(motor_kybrd[motor])
        self.arr[i] = int(intensity)*10 + 1500
        self.motor_cmd.data = self.arr
        self.motor_pub.publish(self.motor_cmd)
        return True

    def kill_all(self):
        self.arr = [1500,1500,1500,1500,1500,1500,1500,1500]

        self.motor_cmd.data = self.arr
        self.motor_pub.publish(self.motor_cmd)
        return True
        print "killed all"

    def get_motor_cmd(self):
        inpt = raw_input("enter command [command, intensity]\n")
        if inpt[0] == 's':
            self.kill_all()

        #TODO max acceptable values
        if len(inpt) < 2 or len(inpt) > 5:
            return False
        #seperate power and cmd values
        else:
            cmd = inpt[0]
            pwr = inpt[1:]
            if not cmd.islower():
                return False
            if not int(pwr):
                return False
            return [inpt[0], int(pwr)]


def main():
    #motor object
    m = Motor_Controller()

    while not rospy.is_shutdown():
        #try if inuput is valid
        try:
            [cmd,pwr] = m.get_motor_cmd()
            cmd_error = False
        except (TypeError):
            print "get cmd exception"
            cmd_error = True

        #single motor try block
        if not cmd_error:
            #try if key exists
            try:
                motor_kybrd[cmd]
                m.single_motor(cmd, pwr)
            except (KeyError):
                pass



if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Motor_Controller')
    main()
