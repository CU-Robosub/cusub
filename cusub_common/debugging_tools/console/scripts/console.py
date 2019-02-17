#!/usr/bin/env python
import termios, fcntl, sys, os
import contextlib
import rospy
import math
import time
import tkinter as tk
import tkFont
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from pololu_controller.msg import MotorCommand
from sensor_msgs.msg import Imu
from tkinter import *
from dynamic_reconfigure.msg import *


class Console(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid(row = 4, column = 4)
        self.createROT()
        self.yaw_sub = rospy.Subscriber('/local_control/pid/yaw/state', Float64, self.yaw_callback, queue_size=1)
        self.drive_sub = rospy.Subscriber('/local_control/pid/drive/state', Float64, self.yaw_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/local_control/pid/depth/state', Float64, self.yaw_callback, queue_size=1)
        self.strafe_sub = rospy.Subscriber('/local_control/pid/strafe/state', Float64, self.yaw_callback, queue_size=1)

        self.c = [0,0,0,0]

        self.yaw_pub = rospy.Publisher('/yaw_PID/parameter_updates',Config,queue_size=1)
        self.depth_pub = rospy.Publisher('/depth_PID/parameter_updates',Config,queue_size=1)
        self.yaw_pub = rospy.Publisher('/drive_PID/parameter_updates',Config,queue_size=1)
        self.yaw_pub = rospy.Publisher('/strafe_PID/parameter_updates',Config,queue_size=1)
        self.config_data = Config()

    def createROT(self):
        my_font = tkFont.Font(size=24)
        self.tkvar = StringVar()
        self.p = tk.Entry(self, font=my_font)
        self.i = tk.Entry(self, font=my_font)
        self.d = tk.Entry(self, font=my_font)
        # on change dropdown value
        def submit_callback():
            print("P:{0}".format(self.p.get()))
            print("I:{0}".format(self.i.get()))
            print("D:{0}".format(self.d.get()))
            degree = self.tkvar.get()
            if(degree == 'yaw'):
                c = self.c[0]
                self.c[0]+=3
            if(degree == 'depth'):
                c = self.c[1]
                self.c[1]+=3
            if(degree == 'drive'):
                c = self.c[2]
                self.c[2]+=3
            if(degree == 'strafe'):
                c = self.c[3]
                self.c[3]+=3
            param = DoubleParameter()
            param.name = "Kp"
            param.value = float(self.p.get())
            self.config_data.doubles.append(param)
            param = DoubleParameter()
            param.name = "Ki"
            param.value = float(self.i.get())
            self.config_data.doubles.append(param)
            param = DoubleParameter()
            param.name = "Kd"
            param.value = float(self.d.get())
            self.config_data.doubles.append(param)

            if(degree == 'yaw'):
                self.yaw_pub.publish(self.config_data)
            if(degree == 'drive'):
                self.drive_pub.publish(self.config_data)
            if(degree == 'depth'):
                self.depth_pub.publish(self.config_data)
            if(degree == 'strafe'):
                self.strafe_pub.publish(self.config_data)


        def change_dropdown(*args):
            print( self.tkvar.get() )

        # Dictionary with options
        choices = {'Choose one', 'yaw','depth','drive','strafe',}
        self.tkvar.set('Choose one') # set the default option

        popupMenu = OptionMenu(self, self.tkvar, *choices)
        # Label(self, text="Choose a dish").grid(row = 1, column = 1)
        popupMenu.grid(row = 0, column =1)
        # link function to change dropdown
        self.tkvar.trace('w', change_dropdown)

        self.p_label = tk.Label(self, text='P\n',font=my_font)
        self.p_label.grid(row = 1, column = 1)
        self.p.grid(row = 1, column = 2,sticky=tk.N)


        self.i_label = tk.Label(self, text='I\n',font=my_font)
        self.i_label.grid(row = 2, column = 1)
        self.i.grid(row = 2, column = 2,sticky=tk.N)

        self.d_label = tk.Label(self, text='D\n',font=my_font)
        self.d_label.grid(row = 3, column = 1)
        self.d.grid(row = 3, column = 2,sticky=tk.N)

        self.sumbitButton = tk.Button(self, text='Submit',command=submit_callback)
        self.sumbitButton.grid()
        self.quitButton = tk.Button(self, text='Quit', command=self.quit)
        self.quitButton.grid()

    def roll_callback(self,msg):
        self.roll.insert('0.0',str(msg.data)+'\n')
    def pitch_callback(self,msg):
        self.pitch.insert('0.0',str(msg.data)+'\n')
    def yaw_callback(self,msg):
        self.yaw.insert('0.0',str(msg.data)+'\n')
def main():
    rospy.init_node('Console')
    c = Console()
    c.master.title('Debugging Console')
    c.mainloop()

if __name__ == "__main__":
    main()
