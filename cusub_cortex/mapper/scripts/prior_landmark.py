#!/usr/bin/env python
'''
This script pulls priors from our config yaml and creates markers for Rviz to plot
'''

import rospy
import tf
import yaml
from visualization_msgs.msg import Marker, MarkerArray

class PriorLandmarker():
    def __init__(self):
        self.pub_rate = 10
        self.config_file = "../../../cusub_cortex/state_machine/config/mission_config_C_WINNING.yaml"

        self.markers_pub = rospy.Publisher('cusub_cortex/mapper/prior_markers', MarkerArray, queue_size=10)

        with open(self.config_file,'r') as stream:
            try:
                conf_dict = yaml.safe_load(stream)
            except YAMLError as exc:
                print(exc)

        sg_pri = conf_dict["tasks"]["start_gate"]["prior"]
        ji_pri = conf_dict["tasks"]["jiangshi"]["prior"]
        tri_pri = conf_dict["tasks"]["triangle"]["prior"]
        drop_pri = conf_dict["tasks"]["dropper"]["prior"]
        oct_pri = conf_dict["tasks"]["octagon"]["prior"]
        # create landmark markers#

        self.sg_marker = Marker()
        self.sg_marker.pose.position.x = sg_pri[0]
        self.sg_marker.pose.position.y = sg_pri[1]
        self.sg_marker.pose.position.z = sg_pri[2]
        self.sg_marker.color.a = 1.0 #make not invisible
        self.sg_marker.header.frame_id = "leviathan/description/odom"
        self.sg_marker.scale.x = 1
        self.sg_marker.scale.y = 1
        self.sg_marker.scale.z = 1
        self.sg_marker.mesh_resource = "package://cusub_sim/robosub_descriptions/models/StartGate/StartGate.dae"
        self.ji_marker = Marker()
        self.ji_marker.pose.position.x = ji_pri[0]
        self.ji_marker.pose.position.y = ji_pri[1]
        self.ji_marker.pose.position.z = ji_pri[2]
        self.ji_marker.color.a = 1.0 #make not invisible
        self.ji_marker.header.frame_id = "leviathan/description/odom"
        self.ji_marker.scale.x = 1
        self.ji_marker.scale.y = 1
        self.ji_marker.scale.z = 1
        self.ji_marker.mesh_resource = "package://cusub_sim/robosub_descriptions/models/buoy_jianshi/buoy.dae"
        self.tri_marker = Marker()
        self.tri_marker.pose.position.x = tri_pri[0]
        self.tri_marker.pose.position.y = tri_pri[1]
        self.tri_marker.pose.position.z = tri_pri[2]
        self.tri_marker.color.a = 1.0 #make not invisible
        self.tri_marker.header.frame_id = "leviathan/description/odom"
        self.tri_marker.scale.x = 1
        self.tri_marker.scale.y = 1
        self.tri_marker.scale.z = 1
        self.tri_marker.mesh_resource = "package://cusub_sim/robosub_descriptions/models/triangular_buoy/triangular_buoy.dae"
        self.drop_marker = Marker()
        self.drop_marker.pose.position.x = drop_pri[0]
        self.drop_marker.pose.position.y = drop_pri[1]
        self.drop_marker.pose.position.z = drop_pri[2]
        self.drop_marker.color.a = 1.0 #make not invisible
        self.drop_marker.header.frame_id = "leviathan/description/odom"
        self.drop_marker.scale.x = 1
        self.drop_marker.scale.y = 1
        self.drop_marker.scale.z = 1
        self.drop_marker.mesh_resource = "package://cusub_sim/robosub_descriptions/models/dropper_garlic/dropper_garlic.dae"
        self.oct_marker = Marker()
        self.oct_marker.pose.position.x = oct_pri[0]
        self.oct_marker.pose.position.y = oct_pri[1]
        self.oct_marker.pose.position.z = oct_pri[2]
        self.oct_marker.color.a = 1.0 #make not invisible
        self.oct_marker.header.frame_id = "leviathan/description/odom"
        self.oct_marker.scale.x = 1
        self.oct_marker.scale.y = 1
        self.oct_marker.scale.z = 1
        self.oct_marker.mesh_resource = "package://cusub_sim/robosub_descriptions/models/Octagon/Octagon.dae"

    def add_time(self, marker):
        marker.header.stamp = rospy.get_rostime()

    def pub_landmarks(self):
        r = rospy.Rate(self.pub_rate)
        
        while not rospy.is_shutdown():
            self.add_time(self.sg_marker)
            self.add_time(self.ji_marker)
            self.add_time(self.tri_marker)
            self.add_time(self.drop_marker)
            self.add_time(self.oct_marker)
            markers = MarkerArray()
            markers.markers.append(self.sg_marker)
            markers.markers.append(self.ji_marker)
            markers.markers.append(self.tri_marker)
            markers.markers.append(self.drop_marker)
            markers.markers.append(self.oct_marker)
            
            self.markers_pub.publish(markers)
            print("...pub")
            r.sleep()

    

def main():
    rospy.init_node("prior_landmark_markers")
    pl = PriorLandmarker()
    pl.pub_landmarks()
    print("shutdown")

if __name__ == '__main__':
    main()