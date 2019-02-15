#!/usr/bin/env python
import rospy
import tf
import numpy as np

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from geometry_msgs.msg import PoseArray, Pose
# from tf.transformations import quaternion_from_euler

class Localizer():
    def __init__(self):
        self.pose_pub = rospy.Publisher('/Global_State/task_poses', PoseArray, queue_size=10)
        self.darknet_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.box_to_pose)

    def box_to_pose(self, msg):
        # constants to use
        # TODO: read in from camera_info topic?
        centpixcam = (752/2, 480/2)
        flength = 613.0667
        for box in msg.bounding_boxes:

            if 'dice' in box.Class:
                real_width = 0.2286 # the actual width of the obstacle
            elif box.Class == 'start_gate_flag':
                real_width = 0.381
	    elif box.Class == 'start_gate_pole':
		real_width = 0.0762

            width = box.xmax-box.xmin
            height = box.ymax-box.ymin

            if width/height > 1.5 or width/height < 0.5: # ratio is not correct
                continue

            # center in pixels

            xavg = (box.xmax+box.xmin)/2
            yavg = (box.ymax+box.ymin)/2
            centpixbox = (xavg, yavg)

            realdist = (real_width*flength) / width

            ppm = width / real_width # pixels to meter

            diffcam = ((centpixbox[0] - centpixcam[0])/ppm , (centpixbox[1] - centpixcam[1])/ppm)
            theta = np.arctan(realdist/diffcam[0]) # TODO: should this be flipped?
            thetaquat = tf.transformations.quaternion_from_euler(0,0,theta,axes='sxyz')

            # publish to pose array
            posearray = PoseArray()
            pose = Pose()
            posearray.header.frame_id = box.Class + "," + msg.image_header.frame_id
            posearray.header.stamp = rospy.Time.now()

            pose.position.x = realdist
            pose.position.y = -diffcam[0] # flipped on 9/29
            pose.position.z = diffcam[1]

            pose.orientation.x = thetaquat[0]
            pose.orientation.y = thetaquat[1]
            pose.orientation.z = thetaquat[2]
            pose.orientation.w = thetaquat[3]

            posearray.poses = [pose]
            self.pose_pub.publish(posearray)


def main():
    rospy.init_node('localizer')
    l = Localizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
