"""
Searching Algorithms

Approach a prior for a task, once we get a few yolo hits on search_topic quit out

halfhalf

"""
from tasks.task import Objective
import smach
from geometry_msgs.msg import Point, PoseStamped, Pose
import rospy
import tf

POSE_INDEX = 0
WAIT_INDEX = 1

class Search(Objective):
    """ Search state that implements an indicated search algorithm """
    
    outcomes = ['found','not_found'] # We found task or timed out

    def __init__(self, prior_pose_param, search_topic, num_quit_poses=5, darknet_cameras=[1,1,0,0,1,0]):
        """
        Search objective initialization function

        Parameters
        ---------
        prior_pose_param : str
             Param name of the prior for the task
        search_topic : str
             Topic name to listen for task poses
        num_quit_poses : int
             Number of poses to receive before aborting and transitioning
        darknet_cameras : bool list, len 6
            Darknet cameras to use
        """
        rospy.loginfo("loading search")
        self.listener = tf.TransformListener() # Transform prior into odom
        self.prior_pose_param = prior_pose_param
        rospy.Subscriber(search_topic, PoseStamped, self.exit_callback)
        self.num_quit_poses = num_quit_poses
        self.num_poses_received = 0
        self.darknet_config = darknet_cameras

        super(Search, self).__init__(self.outcomes, "Search")

    def exit_callback(self, msg): # Abort on the first publishing
        self.num_poses_received += 1
        if not self.replan_requested() and self.num_poses_received > self.num_quit_poses:
            rospy.loginfo("Task Found!")
            self.request_replan()

    def execute(self, userdata):
        rospy.loginfo("---Executing Search")
        prior = self.get_odom_prior(self.prior_pose_param)

        self.configure_darknet_cameras(self.darknet_config)
        rospy.sleep(1) # in case we span in the spot of the prior, leave time to gather a few object poses
        if self.go_to_pose(prior, userdata.timeout_obj):
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "not_found"
            else:
                return "found"
        else:
            rospy.logerr("Search unable to find task.")
            userdata.outcome = "not_found"
            return "not_found"

    def get_odom_prior(self, rosparam_str):
        if not rospy.has_param(rosparam_str):
            raise("Could not locate rosparam: " + rosparam_str)
        else:
            xyzframe_list = rospy.get_param(rosparam_str)
            p = PoseStamped()
            p.pose.position.x = xyzframe_list[0]
            p.pose.position.y = xyzframe_list[1]
            p.pose.position.z = xyzframe_list[2]
            if len(xyzframe_list) == 4:     # prior needs to be transformed
                rospy.loginfo("...transforming prior pose to odom")
                p.header.frame_id = xyzframe_list[3]
                p.header.stamp = rospy.Time()
                if not rospy.has_param("~robotname"):
                    raise("Launch file must specify private param 'robotname'")
                try:
                    odom_frame = '/'+ rospy.get_param("~robotname") + '/description/odom'
                    rospy.loginfo("...waiting for transform: " + odom_frame + " -> /" + xyzframe_list[3])
                    self.listener.waitForTransform(p.header.frame_id, odom_frame, p.header.stamp, rospy.Duration(5))
                    rospy.loginfo("...found transform")
                    p = self.listener.transformPose(odom_frame, p)
                except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
                    rospy.logerr(e)
            return p.pose

