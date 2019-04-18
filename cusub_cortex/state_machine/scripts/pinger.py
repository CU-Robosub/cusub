#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String

s = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

pub = rospy.Publisher("/pinger", String)

data = ""
while(1):
  try:
    data += s.read()
    if(data.find(">") != -1):
      pars = data.split("<")[1].split(">")[0]
      ang = float(pars.split(":")[1])
      if ang < 0.0:
        pub.publish("Left")
      else:
        pub.publish("Right")
      data = ""
  except:
    pass
