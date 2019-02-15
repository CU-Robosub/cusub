#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image

def passthrough(occamimg):
    frame = occamimg.header.seq
    if (frame % 180) == 0:
        yoloimg = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        yoloimg.publish(occamimg)

def main():
    rospy.init_node('occam2cam')
    rospy.Subscriber('/occam/image0', Image, passthrough)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
