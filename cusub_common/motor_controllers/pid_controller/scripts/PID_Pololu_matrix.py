#!/usr/bin/env python
from __future__ import division
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from pololu_controller.msg import MotorCommand
from nav_msgs.msg import Odometry


# * Front: 0
# * front_right: 6
# * front_left: 2
# * back_right: 3
# * back_left: 5
# * back: 4
# * left: 9
# * right: 8


class PID_Pololu():
    ## The constructor.
    def __init__(self):

        self.namespace = rospy.get_param("~namespace")

        self.cmd_data = Float64MultiArray
        #self.effort_array = np.zeros([1,6])
        self.effort_array = [0, 0, 0, 0, 0, 0]
        self.motor_array = np.empty([8,6])

        '''roll, pitch, yaw, depth, drive, strafe'''
        self.motor_array[0] = [ 1,  1,    0, -.5,   0,   0] #Front Right
        self.motor_array[1] = [-1,  1,    0, -.5,   0,   0] #Front Left
        self.motor_array[2] = [ 1, -1,    0, -.5,   0,   0] #Back Right
        self.motor_array[3] = [-1, -1,    0, -.5,   0,   0] #Back Left
        self.motor_array[4] = [ 0,  0, -0.5,   0,   0,  .5] #Front
        self.motor_array[5] = [ 0,  0,  0.5,   0,   0,  .5] #Back
        self.motor_array[6] = [ 0,  0,  0.2,   0, -.8,   0] #Left -.65
        self.motor_array[7] = [ 0,  0, -0.2,   0, -.8,   0] #Right -.8

        '''FR, FL, BR, BL,  F,  B,  L,  R'''
        self.flip_motor_array = np.array([1, 1, 1, 1, 1, 1, 1, 1])
        self.scale_array      = np.array([5, 5, 5, 5, 5, 5, 5, 5])
        self.offset_array     = np.array([1420, 1420, 1420, 1420,
                                          1500, 1500, 1500, 1500])

        ## Publisher for sending commands to the pololu
        self.motor_pub = rospy.Publisher('/drivers/pololu_control/command',
                                         Float64MultiArray, queue_size=1)

        rospy.Timer(rospy.Duration(.02), self.motor_publish)

        ## The Float array to be sent to the pololu command
        self.cmd_data = Float64MultiArray()
        ## Subscriber for the roll pid topic
        self.roll_sub = rospy.Subscriber(self.namespace + '/local_control/pid/roll/control_effort',
                                         Float64, self.roll_callback)
        ## Subscriber for the pitch pid topic
        self.pitch_sub = rospy.Subscriber(self.namespace + '/local_control/pid/pitch/control_effort',
                                          Float64, self.pitch_callback)
        ## Subscriber for the yaw pid topic
        self.yaw_sub = rospy.Subscriber(self.namespace + '/local_control/mux/yaw/control_effort',
                                        Float64, self.yaw_callback)
        ## Subscriber for the depth pid topic
        self.depth_sub = rospy.Subscriber(self.namespace + '/local_control/pid/depth/control_effort',
                                          Float64, self.depth_callback)
        ## Subscriber for the drive pid topic
        self.drive_sub = rospy.Subscriber(self.namespace + '/local_control/pid/drive/control_effort',
                                          Float64, self.drive_callback)
        ## Subscriber for the strafe pid topic
        self.strafe_sub = rospy.Subscriber(self.namespace + '/local_control/pid/strafe/control_effort',
                                           Float64, self.strafe_callback)


        self.yaw_setpoint_pub = rospy.Publisher(self.namespace + '/local_control/pid/yaw/setpoint',
                                                Float64,queue_size=1)
        self.yaw_setpoint_pub_data = Float64()

        print "waiting for yaw"
        msg = rospy.wait_for_message('/sensor_fusion/odometry/filtered', Odometry)

        orientation =  msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x,
                                                                       orientation.y,
                                                                       orientation.z,
                                                                       orientation.w])
        self.yaw_setpoint_pub_data = yaw
        self.yaw_setpoint_pub.publish(self.yaw_setpoint_pub_data)
        print "set yaw setpoint"


    def roll_callback(self,msg): self.effort_array[0] = msg.data

    def pitch_callback(self,msg): self.effort_array[1] = msg.data

    def yaw_callback(self,msg): self.effort_array[2] = msg.data

    def depth_callback(self,msg): self.effort_array[3] = msg.data

    def drive_callback(self,msg): self.effort_array[4] = msg.data

    def strafe_callback(self,msg): self.effort_array[5] = msg.data

    def motor_publish(self, event):
        motor_transform =   np.sum(self.effort_array * self.motor_array, 1)
        motor_transform = motor_transform * self.scale_array
        motor_transform = motor_transform * self.flip_motor_array
        motor_transform = motor_transform + self.offset_array

        command_order = [ 4, 0, 1, 2, 3, 5, 6, 7]
        arr = []
        for i in command_order:
            arr.append(motor_transform[i])

        self.cmd_data.data = arr
        self.motor_pub.publish(self.cmd_data)

    def kill_motors(self):
        self.cmd_data.data = [1500]*8
        self.motor_pub.publish(self.cmd_data)
        print "kill motors"

def main():
    rospy.init_node('PID_Pololu')
    motor = PID_Pololu()
    rospy.on_shutdown(motor.kill_motors)
    rospy.spin()


if __name__ == '__main__':
    main()
