"""
Maps and landmarks for multisub map building
"""

from visualization_msgs.msg import Marker, MarkerArray
class MapLandmark(object):

    MAPPED_POSES = 10

    def __init__(self, name, marker_id, mesh):

        self.name = name
        self.filter = ExpWeightedAvg(5)
        self.marker = Marker()

        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = mesh
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1

        self.marker.header.frame_id = 'world' # TODO sub map frame
        self.marker.ns = '' # todo robot_name
        self.marker.id = marker_id
        self.marker.action = Marker.ADD
        self.marker.color.a = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

    def update_pose(self, pose_stamped):

        if not self.is_mapped():
            self.filter.add_new_pose(pose_stamped.pose)
            if self.is_mapped():

                marker_pose = self.filter.avg_pose

                # special case for startgate
                if self.name == "start_gate":

                    # Pose is centered at 0, fix
                    marker_pose.position.z += 0.6

                    # Has to be upright and rotated 90deg
                    quat = marker_pose.orientation
                    _, _, y = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, y + 1.57)
                    marker_pose.orientation.x = x
                    marker_pose.orientation.y = y
                    marker_pose.orientation.z = z
                    marker_pose.orientation.w = w

                self.marker.pose = marker_pose

    def is_mapped(self):
        if self.filter.num_poses > self.MAPPED_POSES:
            return True
        return False

class Map(object):
    landmarks = {}
    
    def __init__(self):
        self.map_markers_pub = rospy.Publisher('cusub_cortex/mapper/map_markers', MarkerArray, queue_size=10)

    def add_landmark(self, landmark):
        self.landmarks[landmark.name] = landmark

    def update_landmark(self, name, pose):
        if name in self.landmarks.keys():
            self.landmarks[name].update_pose(pose)

    def publish_mapped(self):

        markers = MarkerArray()

        for _, landmark in self.landmarks.iteritems():
            if landmark.is_mapped():
                marker = landmark.marker
                marker.header.stamp = rospy.get_rostime()
                markers.markers.append(marker)

        self.map_markers_pub.publish(markers)

"""
dice1_landmark = MapLandmark("dice1", 1, "package://robosub_descriptions/models/Dice1/Dice1.dae")
        dice2_landmark = MapLandmark("dice2", 2, "package://robosub_descriptions/models/Dice2/Dice2.dae")
        dice5_landmark = MapLandmark("dice5", 3, "package://robosub_descriptions/models/Dice5/Dice5.dae")
        dice6_landmark = MapLandmark("dice6", 4, "package://robosub_descriptions/models/Dice6/Dice6.dae")
        start_gate_landmark = MapLandmark("start_gate", 5, "package://robosub_descriptions/models/StartGate/StartGate.dae")

        self.map = Map()
        self.map.add_landmark(dice1_landmark)
        self.map.add_landmark(dice2_landmark)
        self.map.add_landmark(dice5_landmark)
        self.map.add_landmark(dice6_landmark)
        self.map.add_landmark(start_gate_landmark)
"""

"""
This mapper is 200 lines long, betcha I can write it in <50
"""