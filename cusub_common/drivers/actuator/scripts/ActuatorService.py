#!/usr/bin/env python
import serial
import time
import rospy
import crc8
import struct

from actuator.srv import ActivateActuator

class ActuatorService():

    cuprint = CUPrint("Actuator Service")

    def __init__(self):
        self.ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI02RNUO-if00-port0', 115200)

    def activateActuator(self, req):

        pin = req.actuatorNumber        # 1-6
        timeOn = req.activationTime     # ms

        self.cuprint("Turning on acuator" + bcolors.HEADER + str(pin) + bcolors.ENDC + "for " + bcolors.HEADER + str(timeOn) + bcolors.ENDC +" ms")

        #pin += timeOn
        # I'm not 100% how this code worked, but for now for I will do following
        # <ActuatorNumber><Time_miliseconds>
        # ex Actuator #5 for 210ms write the folloing to serial port
        # 5210

        #cmd_str = ""#str(pin)+str(timeOn)
        #self.ser.write(cmd_str.encode())

	actuator = (pin + 1) | ((pin + 1) << 4)

	time = int(timeOn * (60.0/1000.0))

	hash = crc8.crc8()
	msg = struct.pack("BB", actuator, time)
	hash.update(msg)
	framed = b"\xC0" + msg + hash.digest() + "\xC0"
	print(framed.encode("hex"))
	self.ser.write(framed)


	return []

    def run(self):

        # service to activate actuator
        s = rospy.Service('activateActuator', ActivateActuator, self.activateActuator)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ActuatorService')
    a = ActuatorService()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("Actuator Service has died!");
