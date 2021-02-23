#!/usr/bin/env python
import rospy
import numpy as np
import yaml
import os, rospkg

from sensor_msgs.msg import Image,CameraInfo


"""
This file aligns timestamps of left and right camera so they can be used in stereo cam odometry
"""
rospack = rospkg.RosPack()

def parse_yaml(filename): 
    """
    Creates CameraInfo message with data from yaml file
    """
    stream = open(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info

class CamAdjust:
    def __init__(self):
        self.time_secs = None

        self.cam1 = []
        self.cam2 = []
        self.cam1_info = parse_yaml(os.path.join(rospack.get_path("gazebo_drivers"), "config", "camera_8.yaml"))
        self.cam2_info = parse_yaml(os.path.join(rospack.get_path("gazebo_drivers"), "config", "camera_9.yaml"))

        # Adjusted camera publishers
        self.cam1_pub = rospy.Publisher("/leviathan/description/camera_8/image_raw_adj",Image,queue_size=15)
        self.cam2_pub = rospy.Publisher("/leviathan/description/camera_9/image_raw_adj",Image,queue_size=15)
        self.cam1_info_pub = rospy.Publisher("/leviathan/description/camera_8/camera_info_adj",CameraInfo,queue_size=15)
        self.cam2_info_pub = rospy.Publisher("/leviathan/description/camera_9/camera_info_adj",CameraInfo,queue_size=15)
        
        # Subscribe to the camera images
        rospy.Subscriber("/leviathan/description/camera_8/image_raw",Image,self.cam1_callback)
        rospy.Subscriber("/leviathan/description/camera_9/image_raw",Image,self.cam2_callback)
        
        self.run()

    def cam1_callback(self,msg):
        self.cam1.append(msg)

    def cam2_callback(self,msg):
        self.cam2.append(msg)


    def run(self):
        rate = rospy.Rate(1)
        while len(self.cam1) < 3 or len(self.cam2) < 3:
            rate.sleep()

        rate2 = rospy.Rate(5)
        while not rospy.is_shutdown():
            if len(self.cam1) < 3 or len(self.cam2) < 3:
                rate2.sleep()
            c1 = self.cam1.pop(0)
            if c1.header.stamp.secs - self.cam2[0].header.stamp.secs + (c1.header.stamp.nsecs - self.cam2[0].header.stamp.nsecs)*10**(-9) < 0:
                continue
            c2 = self.cam2.pop(0)

            while self.compare_times(c1,c2,self.cam2[0]):
                c2 = self.cam2.pop(0)

            c2.header.stamp = c1.header.stamp
            self.cam1_info.header.stamp = c1.header.stamp
            self.cam2_info.header.stamp = c1.header.stamp

            self.cam1_pub.publish(c1)
            self.cam2_pub.publish(c2)
            self.cam1_info_pub.publish(self.cam1_info)
            self.cam2_info_pub.publish(self.cam2_info)




    def compare_times(self,A,B,C):
        """
        Returns true if A and C are closer in time stamps than A and B
        """
        if np.abs((A.header.stamp.secs - B.header.stamp.secs) + (A.header.stamp.nsecs - B.header.stamp.nsecs)*10**(-9)) >= np.abs((A.header.stamp.secs - C.header.stamp.secs) + (A.header.stamp.nsecs - C.header.stamp.nsecs)*10**(-9)):
            return True
        return False


rospy.init_node("CamAdjust")



if __name__ == "__main__":
    ca = CamAdjust()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()