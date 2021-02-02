#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Image,CameraInfo

class CamAdjust:
    def __init__(self):
        self.time_secs = None
        self.cam1_ready = False
        self.cam2_ready = False
        self.cam1info_ready = False
        self.cam2info_ready = False
        self.cam1 = []
        self.cam2 = []
        self.cam1_info = []
        self.cam2_info = []
        self.cam1_pub = rospy.Publisher("/leviathan/description/camera_8/image_raw_adj",Image,queue_size=15)
        self.cam2_pub = rospy.Publisher("/leviathan/description/camera_9/image_raw_adj",Image,queue_size=15)
        self.cam1_info_pub = rospy.Publisher("/leviathan/description/camera_8/camera_info_adj",CameraInfo,queue_size=15)
        self.cam2_info_pub = rospy.Publisher("/leviathan/description/camera_9/camera_info_adj",CameraInfo,queue_size=15)
        
        rospy.Subscriber("/leviathan/description/camera_8/image_raw",Image,self.cam1_callback)
        rospy.Subscriber("/leviathan/description/camera_9/image_raw",Image,self.cam2_callback)
        rospy.Subscriber("/leviathan/description/camera_8/camera_info_2",CameraInfo,self.cam1_info_callback)
        rospy.Subscriber("/leviathan/description/camera_9/camera_info_2",CameraInfo,self.cam2_info_callback)


        self.run()

    # def cam1_callback(self,msg):
    #     if not (self.cam2_ready and self.cam1info_ready and self.cam2info_ready):
    #         return
    #     cam_2 = self.cam2
    #     cam_1info = self.cam1_info
    #     cam_2info = self.cam2_info
    #     print('')
    #     print(cam_2.header.stamp.secs-msg.header.stamp.secs + (cam_2.header.stamp.nsecs-msg.header.stamp.secs)*10**(-9))
    #     print(cam_1info.header.stamp.secs-msg.header.stamp.secs + (cam_1info.header.stamp.nsecs-msg.header.stamp.secs)*10**(-9))
    #     print(cam_2info.header.stamp.secs-msg.header.stamp.secs + (cam_2info.header.stamp.nsecs-msg.header.stamp.secs)*10**(-9))
    #     cam_2.header = msg.header
    #     cam_1info.header = msg.header
    #     cam_2info.header = msg.header
    #     self.cam1_pub.publish(msg)
    #     self.cam2_pub.publish(cam_2)
    #     self.cam1_info_pub.publish(cam_1info)
    #     self.cam2_info_pub.publish(cam_2info)
    def cam1_callback(self,msg):
        if not self.cam1info_ready:
            return
        self.cam1_ready = True
        self.cam1.append(msg)

    def cam2_callback(self,msg):
        if not self.cam2info_ready:
            return
        self.cam2_ready = True
        self.cam2.append(msg)

    def cam1_info_callback(self,msg):
        self.cam1info_ready = True
        self.cam1_info.append(msg)

    def cam2_info_callback(self,msg):
        self.cam2info_ready = True
        self.cam2_info.append(msg)

    def run(self):
        rate = rospy.Rate(1)
        while not (self.cam1_ready and self.cam2_ready and self.cam1info_ready and self.cam2info_ready):
            rate.sleep()
        rate.sleep()
        rate.sleep()
        rate.sleep()
        rate2 = rospy.Rate(5)
        while not rospy.is_shutdown():
            c1 = self.cam1.pop(0)
            if c1.header.stamp.secs - self.cam2[0].header.stamp.secs + (c1.header.stamp.nsecs - self.cam2[0].header.stamp.nsecs)*10**(-9) < 0:
                continue
            c2 = self.cam2.pop(0)
            c1i = self.cam1_info.pop(0)
            c2i = self.cam2_info.pop(0)
            while self.compare_times(c1,c2,self.cam2[0]):
                c2 = self.cam2.pop(0)
            while self.compare_times(c1,c1i,self.cam1_info[0]):
                c1i = self.cam1_info.pop(0)
            while self.compare_times(c1,c2i,self.cam2_info[0]):
                c2i = self.cam2_info.pop(0)
            # print('')
            # print(c1.header.stamp.secs-c2.header.stamp.secs + (c1.header.stamp.nsecs-c2.header.stamp.nsecs)*10**(-9))
            t1 = c1.header.stamp.secs-c1i.header.stamp.secs + (c1.header.stamp.nsecs-c1i.header.stamp.nsecs)*10**(-9)
            t2 = c1.header.stamp.secs-c2i.header.stamp.secs + (c1.header.stamp.nsecs-c2i.header.stamp.nsecs)*10**(-9)
            # if t1 > 5:
            #     print('')
            #     print(t1)
            #     print('vs')
            #     print(c1.header.stamp.secs-self.cam1_info[0].header.stamp.secs + (c1.header.stamp.nsecs-self.cam1_info[0].header.stamp.nsecs)*10**(-9))
            #     print('')
            # else:
            #     print(t1)
            # if t2 > 5:
            #     print('')
            #     print(t2)
            #     print('vs')
            #     print(c1.header.stamp.secs-self.cam2_info[0].header.stamp.secs + (c1.header.stamp.nsecs-self.cam2_info[0].header.stamp.nsecs)*10**(-9))
            # else:
            #     print(t2)
            c2.header.stamp = c1.header.stamp
            c1i.header.stamp = c1.header.stamp
            c2i.header.stamp = c1.header.stamp

            self.cam1_pub.publish(c1)
            self.cam2_pub.publish(c2)
            self.cam1_info_pub.publish(c1i)
            self.cam2_info_pub.publish(c2i)
            while(len(self.cam1) < 3 or len(self.cam2) < 3):
                x = 1



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