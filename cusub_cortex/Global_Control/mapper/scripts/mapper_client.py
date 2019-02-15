#! /usr/bin/env python
import rospy
import actionlib
import mapper.msg
import math
import tf
from numpy import linalg as LA
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from numpy import linalg as LA
class Mapper(object):
    # create messages that are used to publish feedback/result
    _feedback = mapper.msg.mapperFeedback()
    _result = mapper.msg.mapperResult()

    def __init__(self, name):
        name = '/mapper'

        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.pose_callback, queue_size=1)
        self.odom = False
        self.yaw_setpoint_pub = rospy.Publisher('/local_control/pid/yaw/setpoint',Float64,queue_size=20)
        self.yaw_setpoint_pub_data = Float64()
        self.depth_setpoint_pub = rospy.Publisher('/local_control/pid/depth/setpoint',Float64,queue_size=20)
        self.depth_setpoint_pub_data = Float64()
        self.drive_setpoint_pub = rospy.Publisher('/local_control/pid/drive/setpoint',Float64,queue_size=20)
        self.drive_setpoint_pub_data = Float64()
        self.strafe_setpoint_pub = rospy.Publisher('/local_control/pid/strafe/setpoint',Float64,queue_size=20)
        self.strafe_setpoint_pub_data = Float64()



        self.yaw_sub = rospy.Subscriber('/yaw_PID/control_effort', Float64, self.yaw_callback)
        self.depth_sub = rospy.Subscriber('/depth_PID/control_effort', Float64, self.depth_callback)
        self.drive_sub = rospy.Subscriber('/drive_PID/control_effort', Float64, self.drive_callback)
        self.strafe_sub = rospy.Subscriber('/strafe_PID/control_effort', Float64, self.strafe_callback)

        self.last_yaw = 0
        self.last_depth = 0
        self.last_drive = 0
        self.last_strafe = 0

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, mapper.msg.mapperAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print("Started {0}".format(name))

    def yaw_callback(self,msg):
        self.last_yaw = msg.data
    def depth_callback(self,msg):
        self.last_depth = msg.data
    def drive_callback(self,msg):
        self.last_drive = msg.data
    def strafe_callback(self,msg):
        self.last_strafe = msg.data

    def pose_callback(self,msg):
        self.odom = msg

    def turn_angle(self,a,b):
        #Create unit Vector for a and b
        bH = np.array([b[0]-a[0],b[1]-a[1]])
        bH = bH/LA.norm(bH)
        aH= np.array([math.sin(a[2]),math.cos(a[2])])
        aH = aH/LA.norm(aH)
        #the arcmath.cos of the dot of the two unit vectors in the angle between them
        d = np.dot(aH,bH)
        A = math.acos(d)
        # print A
        return A

    def execute_cb(self, goal):
        # helper variables
        while(not self.odom):
            print("Waiting on odom")
        success = False
        temp = 1;
        (r, p, goal_yaw) = tf.transformations.euler_from_quaternion([goal.end_point.pose.pose.orientation.x, goal.end_point.pose.pose.orientation.y, goal.end_point.pose.pose.orientation.z, goal.end_point.pose.pose.orientation.w])
        (r, p, current_yaw) = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
        # print("goal Yaw: {0}".format(goal_yaw))
        goal_x = goal.end_point.pose.pose.position.x
        goal_y = goal.end_point.pose.pose.position.y
        goal_z = goal.end_point.pose.pose.position.z
        # print self.odom
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        current_z = self.odom.pose.pose.position.z


        norm_x = goal_x-current_x
        norm_y = goal_y-current_y

        norm = np.array([norm_x,norm_y])
        norm = norm/LA.norm(norm)

        d = np.dot(norm,np.array([1,0]))
        turn = math.acos(d)
        xH = np.array([1,0])
        cross = np.cross(xH,norm)
        turn = (np.sign(cross)*turn)

        distance = math.sqrt((goal_x-current_x)**2+(goal_y-current_y)**2)
        rt = rospy.Rate(30)

        print("Current POS X: {0} Y: {1} Z: {2} Yaw: {3}".format(current_x,current_y,current_z,current_yaw))
        print("Goal POS X: {0} Y: {1} Z: {2} Yaw: {3}".format(goal_x,goal_y,goal_z,goal_yaw))
        print("Turn Angle: {0} Distance: {1}".format(turn,distance))


        self.depth_setpoint_pub.publish(goal_z)
        # while(1):
        #     if(abs(self.last_depth-goal_z)<.2):
        #         break
        #     rt.sleep()

        self.yaw_setpoint_pub_data = turn
        self.yaw_setpoint_pub.publish(self.yaw_setpoint_pub_data);
        while(1):
            if(abs(self.last_yaw-turn)<.2):
                break
            rt.sleep()
        self.drive_setpoint_pub_data = distance
        self.drive_setpoint_pub.publish(self.drive_setpoint_pub_data)

        while(1):
            if(abs(self.last_drive-distance)<.2):
                break
            rt.sleep()
        print self._result
        self._result.finish = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('mapper')
    server = Mapper(rospy.get_name())
    rospy.spin()
