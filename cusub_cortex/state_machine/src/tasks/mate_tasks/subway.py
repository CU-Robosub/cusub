#!/usr/bin/env python
from __future__ import division

import cv2
# import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from copy import copy
import os
import rospy
from sensor_msgs.msg import Image

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result


class Subway:
    def __init__(self,forward_topic,down_topic):
        self.node_name = "subway"
        rospy.Subscriber(forward_topic,Image,self.forward_callback)
        rospy.Subscriber(down_topic,Image,self.down_callback)
        self.forward_image = None
        self.down_image = None
        self.bridge = CvBridge()

        self.saved_names = ["subway_img_1.npy","subway_img_2.npy","subway_img_3.npy","subway_img_4.npy","subway_img_5.npy"]
        self.raw_images = [np.load(self.saved_names[i]) for i in range(5)]
        self.rect_images = [i for i in range(5)]
        self.resize_images = [i for i in range(5)]
        self.stiched_image = None

        self.top_points = [[0,0],[0,0]]
        self.tidx = 0
        self.top_names = ["Top Left", "Bottom Left"]
        self.points = [[0,0] for i in range(4)]
        self.pointName = ['Top Left','Top Right', 'Bottom Right', 'Bottom Left']
        self.pidx = 0

        self.image_names = ['Longside1','Shortside2','Longside3','Shortside4','Topface5']

        self.get()
        self.rot_top()
        self.rect()
        self.resize()
        self.stitch('BeforeRotate')
        self.askRotate()
        self.stitch('Final Stitch')



    def forward_callback(self,msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e
        self.forward_image = np.array(frame, dtype=np.uint8)

    def down_callback(self,msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e
        self.down_image = np.array(frame, dtype=np.uint8)
        


    def get(self):
        while not rospy.is_shutdown():
            side = int(raw_input("Enter subway face  (5 for top, 1-4 sides, 0 for done taking pictures and stitch)\n"))
            name = "subway_img_" + str(side) + ".npy"
            image = None
            if side == 0:
                break
            if not(0 < side <= 5):
                continue
            if side == 5:
                image = self.down_image
            else:
                image = self.forward_image
            self.raw_images[side-1] = image
            np.save(name,image)
        print('Hello')

    def click_event(self,event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:    
            print(x, ' ', y)
            self.points[self.pidx] = [x,y]
            print(self.pointName[self.pidx]+":"+str((x,y)))

            self.pidx = (self.pidx + 1)%4
            if self.pidx == 0:
                print('Below are the current Points. Exit image if ok. Right click to scroll back through points')
                print(self.pointName)
                print(self.points)
            print("Now Click on " + self.pointName[self.pidx])
    
        # checking for right mouse clicks     
        if event==cv2.EVENT_RBUTTONDOWN:
            self.pidx = (self.pidx + 1)%4
            print("Now Click on " + self.pointName[self.pidx])

    def click_event_top(self,event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:    
            print(x, ' ', y)
            self.top_points[self.tidx] = [x,y]
            print(self.top_names[self.tidx]+":"+str((x,y)))

            self.tidx = (self.tidx + 1)%2
            if self.tidx == 0:
                print('Below are the current Points. Exit image if ok.')
                print(self.top_names)
                print(self.top_points)
            print("Now Click on " + self.top_names[self.tidx])
        if event==cv2.EVENT_RBUTTONDOWN:
            self.tidx = (self.tidx + 1)%2
            print("Now Click on " + self.top_names[self.tidx])
    
    
    def rot_top(self):
        img = self.raw_images[4]
        cv2.imshow("Top before rotation",img)
        cv2.setMouseCallback("Top before rotation", self.click_event_top)
        cv2.waitKey()

        x = self.top_points[0][1] - self.top_points[1][1]
        y = self.top_points[0][0] - self.top_points[1][0]

        ang = np.arctan2(y,x)
        ang = -(ang * 180.0)/np.pi

        self.raw_images[4] = rotate_image(self.raw_images[4],ang)



    
    def construct_rect(self,img,pts):
        pts[0][0] = pts[3][0]
        pts[1][0] = pts[2][0]
        width = pts[1][0]-pts[0][0]
        if abs(pts[0][1]-pts[3][1]) >= abs(pts[1][1]-pts[2][1]):
            height = abs(pts[0][1]-pts[3][1])
        else:
            height = abs(pts[2][1]-pts[1][1])
        rect_img = copy(img[:height,:width,:])
        m1 = float(pts[1][1]-pts[0][1])/(pts[1][0]-pts[0][0])
        m2 = float(pts[2][1]-pts[3][1])/(pts[2][0]-pts[3][0])
        for i in range(width):
            top = (pts[0][0]+i,pts[0][1]+int(m1*i))
            bot = (pts[3][0]+i,pts[3][1]+int(m2*i))
            h1 = bot[1]-top[1] 
            diff = height - (h1)
            col = img[top[1]:bot[1],top[0],:]
            if diff == 0:
                # print(col)
                rect_img[:,i,:] = col
                continue
            col_copy = np.zeros((height,3))
            col_copy[:h1,:] = col
            jump =  int(height/diff)
            for j in range(diff):
                col_copy[(jump*j+1):,:] = col_copy[(jump*j):-1,:]
            rect_img[:,i,:] = col_copy
        return rect_img


    def rect(self):
        for i in range(5):
            self.pidx = 0
            img = self.raw_images[i]
            cv2.imshow(self.image_names[i],img)
            cv2.setMouseCallback(self.image_names[i], self.click_event)
            cv2.waitKey()

            self.rect_images[i] = self.construct_rect(img,self.points)

    def resize_helper(self,img,h,w):
        resize1_img = np.zeros((img.shape[0],w,3), np.uint8)

        col_jmp = float(img.shape[1])/w

        for i in range(w):
            resize1_img[:,i,:] = img[:,int(col_jmp*i),:]
        
        resize_img = np.zeros((h,w,3), np.uint8)
        row_jump = float(img.shape[0])/h

        for i in range(h):
            resize_img[i,:,:] = resize1_img[int(i*row_jump),:,:]
        return resize_img   

    def resize(self):
        for i in range(5):
            h = 250
            w = 250 + ((i+1)%2) * 250
            self.resize_images[i] = self.resize_helper(self.rect_images[i],h,w)

    def stitch(self,name):
        stitched_img = np.ones((500,1500,3), np.uint8)*255
        stitched_img[250:,:250,:] = self.resize_images[3]
        stitched_img[250:,250:750,:] = self.resize_images[2]
        stitched_img[:250,250:750,:] = self.resize_images[4]
        stitched_img[250:,750:1000,:] = self.resize_images[1]
        stitched_img[250:,1000:,:] = self.resize_images[0]
        cv2.imshow(name,stitched_img)
        cv2.waitKey()

    def askRotate(self):
        while True:
            rotate = int(raw_input("How much do we need to rotate the top?(Enter 0 for none, 1 for 90, 2 for 180, 3 for 270) All angles are counterclockwise\n"))
            if not(0<= rotate < 4):
                continue
            if rotate == 0:
                break

            for i in range(rotate):
                self.rect_images[4] = np.rot90(self.rect_images[4])
            self.resize_images[4] = self.resize_helper(self.rect_images[4],250,500)
            
            cv2.imshow('New top face',self.resize_images[4])
            cv2.waitKey()

            good = -1
            while not(good==0 or good==1):
                good = int(raw_input("Is this good? 1 for yes 0 for no\n"))
            if good == 1:
                break






            
        



if __name__ == "__main__":
    rospy.init_node("subway_mate")
    print(os.getcwd())
    Subway("/leviathan/description/camera_0/image_raw","/leviathan/description/camera_8/image_raw")

