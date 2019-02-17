#!/usr/bin/env python
import serial
import time
import rospy

class ActuatorService():

    def __init__(self):
        self.ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI02RNUO-if00-port0', 9600)

    def activateActuator(self, req):

        pin = req.actuatorNumber        # 1-6
        timeOn = req.activationTime     # ms

        rospy.loginfo("Turning on acuator %d for %dms" % (pin, timeOn))

        pin += timeOn
        # I'm not 100% how this code worked, but for now for I will do following
        # <ActuatorNumber><Time_miliseconds>
        # ex Actuator #5 for 210ms write the folloing to serial port
        # 5210

        cmd_str = str(pin)+str(timeOn)

        self.ser.write(cmd_str.encode())

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
