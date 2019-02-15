#!/usr/bin/env python
import serial
import time
import rospy


class Actuator():
    """docstring for Actuator"""
    def __init__(self):
        self.ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI02RNUO-if00-port0', 9600)

    def get_input(self):
        while not rospy.is_shutdown():
            pin = str(input("Actuater to activate (1-6): "))
	    if pin == "-1":
		break
            timeOn = str(input("Time to activate pin %s (ms): "%pin))
            print(pin, timeOn)
            pin += timeOn
            self.ser.write(pin.encode())


if __name__ == "__main__":
    rospy.init_node('actuator_activate')
    a = Actuator()
    try:
        a.get_input()
    except rospy.ROSInterruptException:
      pass
