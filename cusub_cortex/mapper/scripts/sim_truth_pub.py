#!/usr/bin/env python

"""
Publishes gazebo's truth data as Detection msgs to the mapper
Configurable via the config/sim_truth.yaml
"""
import sys
import rospy

from localizer.msg import Detection
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Used with internal self.pubs structure
CLASS_INDEX = 0
PUBLISHER_INDEX = 1

class SimTruthPub:
    def __init__(self):
        rospy.loginfo("Sim Truth Publisher Initializing")
        objects = rospy.get_param('sim_pub/objects_to_publish')
        self.pub = rospy.Publisher('cusub_perception/mapper/task_poses', Detection, queue_size=1)

        self.objs = {}        
        for obj_name in objects:
            obj_topic = rospy.get_param('sim_pub/' + obj_name)
            if not isinstance(obj_topic, list): # make it a list if it's not
                obj_topic = [obj_topic]
            for topic in obj_topic:
                    rospy.Subscriber(topic, Odometry, self.callback)
                    self.objs[topic] = obj_name
                
    def callback(self, msg):
        topic = msg._connection_header['topic']
        class_id = self.objs[topic]
        detection = self.odom2Detection(msg, class_id)
        self.pub.publish(detection)

    def odom2Detection(self, odom_msg, class_id):
        det = Detection()
        det.class_id = class_id
        det.pose = PoseStamped()
        det.pose.pose = odom_msg.pose.pose
        det.pose.header = odom_msg.header
        return self.correctPoses(det)
    
    def correctPoses(self, det):
        if "dice" in det.class_id:
            det.pose.pose.position.z -= 0.5
        return det

def main():
    rospy.init_node('sim_truth_pub')
    stp = SimTruthPub()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit()
