"""
Search Objective:
@brief Approach a prior for a task

Two ways to initialize:
1) exit_topic, once a msg has been received on 'exit_topic' quit out
    indicate with target_class = None or call Search.from_topic()
2) target_class, a service request is made that inquires about which bounding boxes are available
    if target_class is among the bounding boxes available, the search quits out
    indicate with target_class != None or call Search.from_bounding_box()
"""
from tasks.task import Objective
import smach
from geometry_msgs.msg import Point, PoseStamped, Pose
import rospy
import tf
from darknet_multiplexer.srv import DarknetClasses

DARKNET_CAMERAS_DEFAULT=[1,1,0,0,1,0]

class Search(Objective):
    
    outcomes = ['found','not_found'] # We found task or timed out

    def __init__(self, prior_pose_param, exit_topic, target_class=None, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """
        Search objective initialization function

        Parameters
        ---------
        prior_pose_param : str
             Param name of the prior for the task
        exit_topic : str
             Topic to listen to and quit out after num_exit_msgs have arrived
        target_class : str
            class to check for when making a service request to check which bounding boxes are available
            if None will use the exit_topic
        darknet_cameras : bool list, len 6
            Darknet cameras to use
        """
        self.listener = tf.TransformListener() # Transform prior into odom
        self.prior_pose_param = prior_pose_param
        if target_class == None:                # Exit by subscribing to a topic
            self.topic_exit = True
            rospy.Subscriber(exit_topic, PoseStamped, self.exit_callback)
            
        else:                                   # Exit by a bounding box with our target class being available
            self.topic_exit = False
            self.target_class = target_class
            self.service = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
            
        self.darknet_config = darknet_cameras
        super(Search, self).__init__(self.outcomes, "Search")
    
    @classmethod
    def from_topic(self, prior_pose_param, exit_topic, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """ Allows the search to quit if a msg is published to exit_topic, default behavior of this class """
        return self(prior_pose_param, exit_topic, darknet_cameras=darknet_cameras)

    @classmethod
    def from_bounding_box(self, prior_pose_param, target_class, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """ Allows the search to quit if darknet is publishing bounding box msgs of the target class at the current time """
        return self(prior_pose_param, None, target_class, darknet_cameras=darknet_cameras)

    def exit_callback(self, msg): # Abort on the first publishing
        if not self.replan_requested():
            rospy.loginfo("Task Found!")
            self.request_replan()

    def execute_topic_exit(self, userdata):
        prior = self.get_odom_prior(self.prior_pose_param)
        rospy.sleep(1) # in case we spawn in the spot of the prior, leave time to gather a few object poses
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

    def execute_bonding_box_exit(self, userdata):
        prior = self.get_odom_prior(self.prior_pose_param)
        self.go_to_pose_non_blocking(prior)

        # Ask the server for available bounding box classes and check if our target_class is among those
        while self.target_class not in self.service(rospy.Duration(1)).classes and not rospy.is_shutdown():
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timed_out"
                return "not_found"
            elif self.check_reached_pose(prior):
                rospy.logerr("Search unable to find task.")
                userdata.outcome = "not_found"
                return "not_found"
        return "found"

    def execute(self, userdata):
        rospy.loginfo("---Executing Search")
        self.configure_darknet_cameras(self.darknet_config)
        
        if self.topic_exit:
            return self.execute_topic_exit(userdata)
        else:
            return self.execute_bonding_box_exit(userdata)

    def get_odom_prior(self, rosparam_str):
        if not rospy.has_param(rosparam_str):
            raise(Exception("Could not locate rosparam: " + rosparam_str))
            
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

