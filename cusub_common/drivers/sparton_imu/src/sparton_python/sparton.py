#!/usr/bin/env python2

import rospy
from pdb import set_trace # python debugger
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D



class spartonDriver(object):

    def __init__(self, port="/dev/sensors/ftdi_FTG5PM3K", baudrate=115200):

        self.sparton_serial = serial(port, baudrate, timeout=.5)

        self.sparton_imu
        self.imu_data = Imu(header=rospy.Header(frame_id="SpartonCompassIMU"))

        #TODO find proper covariences
        self.imu_data.orientation_covariance = [1e-6, 0, 0,
                                                0, 1e-6, 0,
                                                0, 0, 1e-6]

        self.imu_data.angular_velocity_covariance = [1e-6, 0, 0,
                                                     0, 1e-6, 0,
                                                     0, 0, 1e-6]

        self.imu_data.linear_acceleration_covariance = [1e-6, 0, 0,
                                                        0, 1e-6, 0,
                                                        0, 0, 1e-6]
        myStr1='\r\n\r\nprinttrigger 0 set drop\r\n'
        # this is with true north setting
        # myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or yawt_trigger or time_trigger or set drop\r\n'
        # this is output magnetic north
        myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or time_trigger or set drop\r\n'
            # set the number high to get lower update rate , the IMU data is 100Hz rate , the string is 130 byte with 10 bit/byte , the max sampling rate is 88Hz
            # printmodulus 60:10Hz 40:15~17  35:17~18Hz 30:21Hz 25:23~27Hz ,20: 30~35Hz,15:35~55Hz 10: 55~76 Hz,  5: 70~100 Hz, 1:70~100 Hz
        myStr_printmodulus=('printmodulus %i set drop\r\n' % D_Compassprintmodulus  )
        myStr3='printtrigger printmask set drop\r\n'
