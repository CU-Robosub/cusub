"""
Search Objective
"""
from tasks.task import Objective
# import smach
from detection_listener.listener import DetectionListener
from geometry_msgs.msg import PoseStamped
import rospy
import tf
from darknet_multiplexer.srv import DarknetClasses

DARKNET_CAMERAS_DEFAULT=[1,1,1,1,1,0]

class Search(Objective):

    outcomes = ['found','not_found']

    def __init__(self, task_name, target_classes, prior_pose_param_str, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        super(Search, self).__init__(self.outcomes, task_name + "/Search")
        self.darknet_config = darknet_cameras
        self.prior_pose_param_str = prior_pose_param_str
        self.target_classes = target_classes
        self.det_listener = DetectionListener()
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        self.smprint("executing")
        self.smprint("configuring darknet cameras")
        self.configure_darknet_cameras(self.darknet_config)

        prior = self.get_odom_prior(self.prior_pose_param_str) # attempt to grab from mapper first --> we may already have localized it

        userdata.outcome = "success"
        return "found"
        
    def get_odom_prior(self, rosparam_str):
        if not rospy.has_param(rosparam_str):
            raise(Exception("Could not locate rosparam: " + rosparam_str))
            
        xyzframe_list = rospy.get_param(rosparam_str)
        p = PoseStamped()
        p.pose.position.x = xyzframe_list[0]
        p.pose.position.y = xyzframe_list[1]
        p.pose.position.z = xyzframe_list[2]
        if len(xyzframe_list) == 4:     # prior needs to be transformed
            self.smprint("...transforming prior pose to odom")
            p.header.frame_id = xyzframe_list[3]
            p.header.stamp = rospy.Time()
            if not rospy.has_param("~robotname"):
                raise("Launch file must specify private param 'robotname'")
            try:
                odom_frame = '/'+ rospy.get_param("~robotname") + '/description/odom'
                self.smprint("...waiting for transform: " + odom_frame + " -> /" + xyzframe_list[3])
                self.listener.waitForTransform(p.header.frame_id, odom_frame, p.header.stamp, rospy.Duration(5))
                self.smprint("...found transform")
                p = self.listener.transformPose(odom_frame, p)
            except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
                rospy.logerr(e)
        return p.pose

