#!/usr/bin/env python

import sys
import rospy
import rospy, rosparam, roslib

from pololu_controller.pololu_serial import PololuSerial


def main():
    rospy.init_node('pololu_controller')
    baudrate = rospy.get_param(rospy.search_param('baud_rate'))
    paramlist = rospy.get_param(rospy.search_param('paramlist'))
    pololu = PololuSerial(baudrate, paramlist)
    rospy.on_shutdown(pololu.go_home)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pololu.go_home()
        sys.exit()


if __name__ == '__main__':
    main()
