#!/usr/bin/env python2
'''
Will become python module for interfacing with pololu
'''
import sys, os
import re
import struct
import binascii
import serial
import rospy, rosparam, roslib
#TODO wtf is this for?
roslib.load_manifest("rosparam")

# from pololu_controller.msg import MotorCommand
from std_msgs.msg import Float64MultiArray


class PololuSerial(object):
    '''
    for initializing and controlling the pololu

    '''
    def __init__(self, baudrate, paramlist) :
        '''the initialize function,both legacy and currentnode'''
        self.pololu_id = None
        self.pololu_channels = None
        baud = baudrate
        port = self.find_pololu()
        #no pololu, kill node
        if port is None:
            sys.exit()
        self.pololu = serial.Serial(port , baud, timeout=0.5)
        self.pololu.flush()
        #load parameters from yaml and make motor dictionary
        self.motor_list = paramlist[0][0]
        #initialize subscribers and node
        self.legacy_sub = rospy.Subscriber('/drivers/pololu_control/command',
                                         Float64MultiArray,
                                           self.legacy_callback,
                                           queue_size=40)


    def find_pololu(self):
        '''check for pololu device under serial id'''
        srl_dev_dr = "/dev/serial/by-id/"
        sdevices = os.listdir(srl_dev_dr)
        if not sdevices:
            rospy.logerr("no serial device found")
            return None
        for device in sdevices:
            #check for Pololu and serial link 00
            if('Pololu_Mini_Maestro' and 'if00') in device:
                self.pololu_id = re.search(r"[0-9]{8}", device).group()
                self.pololu_channels = int(re.search(r"[0-9]{2}", device).group())
                rospy.loginfo("Pololu ID: %s found", self.pololu_id)
                return srl_dev_dr + device
        rospy.logerr("no pololu serial devices found")
        return None


    def error_handler(self, error):
        '''handles loging incoming errors'''
        return -1

    def legacy_callback(self, msg):
        '''allow controlll of pololu using array 64'''
        arr = [msg.data[0], 0, msg.data[2], msg.data[3], msg.data[5],
               msg.data[4], msg.data[1], 0, msg.data[7], msg.data[6]]

        serialBytes = [0x9f, 10, 0]

        for x in arr:
            pwm = x*4
            pwm = int(pwm)
            serialBytes.append(pwm & 0x7F)
            serialBytes.append((pwm >> 7) & 0x7F)

        self.pololu.write(serialBytes)
        binary_string = binascii.unhexlify(b'A1')
        self.pololu.write(binary_string)
        self.pololu.flush()
        if(self.pololu.in_waiting > 0):
            s = self.pololu.read(2)
            out = struct.unpack('<H', s)

    def set_target_pwm(self, channel, pwm):
        '''take a pwm value and set target'''
        if pwm not in range(999, 2001):
            rospy.logwarn("pwm outside range: %s", pwm)
            return -1
        return self.set_target(channel, pwm*4)


    def set_target(self, channel, target):
        '''set individual target value'''
        serial_bytes = [0x84]
        if channel >= self.pololu_channels:
            rospy.logwarn("chanel outside range %s > %s",
                          channel, self.pololu_channels)
            return -1
        serial_bytes.append(channel)
        serial_bytes.append(target & 0x7F)
        serial_bytes.append((target >> 7) & 0x7F)
        self.pololu.write(serial_bytes)
        self.pololu.flush()
        while self.pololu.in_waiting > 0:
            err = self.pololu.read(2)
            return self.error_handler(err)
        return 0

    def set_multiple_targets(self):
        '''set multiple targets at the same time'''
        pass

    def get_position(self):
        '''3 modes servo, digital output, input configure accordingly'''
        pass

    def get_moving_state(self):
        '''check state of servo'''
        pass

    def get_errors(self):
        '''for flushing errors'''
        pass

    def go_home(self):
        '''should replace kill all function'''
        serial_bytes = [0xA2]
        serial_bytes.append(0)
        serial_bytes.append(0x22)
        self.pololu.flush()
        self.pololu.write(serial_bytes)
        self.pololu.flush()

    def kill_all(self):
        '''set all motors to 1500 Pulse method to be replace by go_home?'''
        arr = [1500, 1500, 1500, 1500, 1500,
               1500, 1500, 1500, 1500, 1500]

        serialBytes = [0x9f, 10, 0]

        for x in arr:
            pwm = x*4
            pwm = int(pwm)
            serialBytes.append(pwm & 0x7F)
            serialBytes.append((pwm >> 7) & 0x7F)

        self.pololu.write(serialBytes)
        binary_string = binascii.unhexlify(b'A1')
        self.pololu.write(binary_string)
        self.pololu.flush()
        if(self.pololu.in_waiting > 0):
            s = self.pololu.read(2)
            out = struct.unpack('<H', s)


def main():
    rospy.init_node('Pololu_Controller')
    pololu = PololuSerial()
    rospy.on_shutdown(pololu.go_home)
    pololu.go_home()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pololu.kill_all()
        sys.exit()

if __name__ == '__main__':
    main()
