#!/usr/bin/env python
import argparse
import numpy as np
import cv2
import sys
import rospy
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image as img
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

# bridge
bridge = CvBridge()


# import matplotlib.pyplot as plt
PY3 = sys.version_info[0] == 3

if PY3:
    xrange = range

#LOCATION:
#takes saturation channel of hsv image, thresholds to find darkest patches
def side_sat(sat):

	sat[sat<100] = 0

	img = cv2.medianBlur(sat,5)
	ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)

	return th1


#takes thresholded sat channel image, finds contours with largest area
#fits 2 largest contours with closest possible bounding boxes
def find_sides(sat, minArea = 1000):
	inv = cv2.bitwise_not(sat)
	im2,contours,hierarchy = cv2.findContours(inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse = True)
	cnt = []
	if cv2.contourArea(contours[0]) >=minArea:
		cnt.append(contours[0])
	if cv2.contourArea(contours[1]) >= minArea:
		cnt.append(contours[1])
	boxes = []
	if cnt is not None:
		for c in cnt:
			rect = cv2.minAreaRect(c)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			# cv2.drawContours(img,[box],0,(0,0,255),2)
			boxes.append(box)
		# cv2.imshow("?", img)
		# cv2.waitKey(0)
	else:
		print("Gate not found!")
	return boxes



def extractmath(bboxes):
	centpixcam = (752/2, 480/2)
	flength = 613.0667
	posepub = rospy.Publisher('/Global_State/task_poses', PoseArray, queue_size=10)
	for box in bboxes:
		w = 0.2286

		xmax = max(box[0][0], box[1][0], box[2][0], box[3][0])
		xmin = min(box[0][0], box[1][0], box[2][0], box[3][0])

		ymax = max(box[0][1], box[1][1], box[2][1], box[3][1])
		ymin = min(box[0][1], box[1][1], box[2][1], box[3][1])


		width = xmax-xmin
		height = ymax-ymin
		if (width/height) > 1.0: # ?
		    continue
		xavg = (xmax+xmin)/2
		yavg = (ymax+ymin)/2
		centpixbox = (xavg, yavg)
		realdist = (w*flength)/width
		ppm = width/w
		diffcam = ((centpixbox[0]-centpixcam[0])/ppm,(centpixbox[1]-centpixcam[1])/ppm)
		theta = np.arctan(realdist/diffcam[0])
		thetaquat = quaternion_from_euler(0, 0, theta, axes='sxyz')
		posearray = PoseArray()
		pose = Pose()
		posearray.header.frame_id = "start_gate"
		posearray.header.stamp = rospy.Time.now()
		pose.position.x = realdist
		pose.position.y = diffcam[0]
		pose.position.z = diffcam[1]
		pose.orientation.x = thetaquat[0]
		pose.orientation.y = thetaquat[1]
		pose.orientation.z = thetaquat[2]
		pose.orientation.w = thetaquat[3]
		posearray.poses=[pose]
		posepub.publish(posearray)
#takes in image from /occam/image0, calls processing and math fns
def gate_finder(image):

    try:
    	img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
    	print(e)


    ## Split the H channel in HSV, and get the saturation channel
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)


    sides = side_sat(s)

    #bounding_boxes contains box2D objects, have ( center (x,y), (width, height), angle of rotation )?
    #[] - bottom left
    #[] - top right
    #[] - top left
    #[] - bottom right
    bounding_boxes = find_sides(sides)



    if len(bounding_boxes) > 1:

    	#get rid of negatives
    	for i in range(len(bounding_boxes)):
    		for j in range(len(bounding_boxes[i])):
    			for k in range(len(bounding_boxes[i][j])):
    				if bounding_boxes[i][j][k] < 0:
    					bounding_boxes[i][j][k] = 0

    	#subtract bottom left from top left for each bounding box:
    	diff1 = abs(bounding_boxes[0][2] - bounding_boxes[0][0])
    	diff2 = abs(bounding_boxes[1][2] - bounding_boxes[1][0])

    	# if difference in x > diff in y delete (horizontal rectangle)
    	if diff1[0] > diff1[1]:
    		bounding_boxes.pop(0)
    	if diff2[0] > diff2[1]:
    		bounding_boxes.pop(1)

		#find halfway pt between midpts

        if len(bounding_boxes) > 1:

    		#find midpts (x)
    		#average top, bottom midpoints
            top_mid = (bounding_boxes[0][1][0]+ bounding_boxes[0][2][0]) / 2
            bot_mid = (bounding_boxes[0][0][0]+ bounding_boxes[0][3][0]) / 2
            mid0 = (top_mid+ bot_mid) / 2

            top_mid = (bounding_boxes[1][1][0]+ bounding_boxes[1][2][0])/ 2
            bot_mid = (bounding_boxes[1][0][0]+ bounding_boxes[1][3][0])/2
            mid1 = (top_mid+ bot_mid)/2

            image_mid = (mid0+ mid1)/2
            mid0 = int(mid0)
            mid1 = int(mid1)
            image_mid = int(image_mid)

            output = img.copy()
            #Draw on image:
            if bounding_boxes is not None:
            	for box in bounding_boxes:
            		cv2.drawContours(output,[box],0,(0,0,255),2)

            cv2.line(output, (mid1, 0) , (mid1, 800), (0, 255, 0))
            cv2.line(output, (mid0, 0) , (mid0, 800), (0, 255, 0))
            cv2.line(output, (image_mid, 0) , (image_mid, 800), (0, 255, 0))

            print("boutta show")
            #cv2.imshow("Output", output)

          
            #cv2.waitKey(0)

            mid_box = [[image_mid + 10, bounding_boxes[0][0][1]],
            			[image_mid-10, bounding_boxes[0][1][1]],
            			[image_mid+10, bounding_boxes[0][2][1]],
            			[image_mid-10, bounding_boxes[0][3][1]]]
            boxes = [bounding_boxes[1], mid_box]

        extractmath(boxes)

def main():
    rospy.init_node('gate_finder')

    rospy.Subscriber('/occam/image0', img, gate_finder)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
