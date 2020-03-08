#!/usr/bin/env python
import rospy
from detection_listener.listener import DetectionListener
"""
Tests the detection tree listener class

Let's listen to the dv's and print the average over the last 30 dvs or 1second
"""

class TestListener:

    def __init__(self):
        self.detection_listener = DetectionListener()

    def run(self):
        r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if self.detection_listener.check_new_dv(0):
                bearing = self.detection_listener.get_avg_bearing(0, num_dv=5)
                # bearing = self.detection_listener.get_avg_bearing(0, secs=0.5)
                if bearing == None:
                    rospy.loginfo("Error encountered")
                else:
                    print(bearing)
                self.detection_listener.clear_new_dv_flag(0)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("detection_listener_test")
    t = TestListener()
    print("running...")
    t.run()