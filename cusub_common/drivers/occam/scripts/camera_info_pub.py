#!/usr/bin/env python2


import rospy
import rosparam
import sys
from sensor_msgs.msg import CameraInfo, Image



class cameraInfoCallback(object):

    def __init__(self, camera = 0 ):
        ''' takes in ros image and publishes camera intrinsics for it '''
        rospy.init_node('occam_camera_info', anonymous=True)
        sub_topic = "/occam/image{}".format(camera)
        self.camera_info = self.load_intrisics(camera = camera)
        rospy.Subscriber(sub_topic, Image, self.camera_info_callback)
        self.pub_info = rospy.Publisher(sub_topic+"/camera_info", CameraInfo, queue_size = 1)
        try:
            rospy.spin()
        except:
            rospy.loginfo("killed the camera info callback")

    def load_intrisics(self, camera):
        ''' loads the intrinsics from the yaml file '''
        f = rospy.get_param("/occam/camera_info_url")
        paramlist = rosparam.load_file(f, default_namespace="occam_params")
        camera_info = CameraInfo()
        camera_info.height =  paramlist[0][0]["image_height"]
        camera_info.width = paramlist[0][0]["image_width"]
        camera_info.distortion_model = paramlist[0][0]["distortion_model"]
        camera_info.D = paramlist[0][0]["distortion_coefficients"]["data"]
        camera_info.K = paramlist[0][0]["camera_matrix"]["data"]
        camera_info.R = paramlist[0][0]["rectification_matrix"]["data"]
        camera_info.P = paramlist[0][0]["projection_matrix"]["data"]
        return camera_info

    def camera_info_callback(self, image):
        ''' take header from camera image and add to camera info message '''
        self.camera_info.header = image.header
        self.pub_info.publish(self.camera_info)
        #print self.camera_info


if __name__ == '__main__':
    camera_num = rospy.get_param("/occam/camera_num")
    info_sub_pub = cameraInfoCallback(camera_num)
