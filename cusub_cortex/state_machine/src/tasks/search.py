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
import actionlib
import tf
import time
from darknet_multiplexer.srv import DarknetClasses
from perception_control.msg import VisualPointAction, VisualPointGoal, VisualPointFeedback

DEFAULT_SEEN_FRAMES = 1
DARKNET_CAMERAS_DEFAULT=[1,1,1,1,1,0]

class Search(Objective):
    
    outcomes = ['found','not_found'] # We found task or timed out

    def __init__(self, prior_pose_param, exit_topic, target_classes=None, target_frames=None,  min_seen_frames=DEFAULT_SEEN_FRAMES, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """
        Search objective initialization function

        Parameters
        ---------
        prior_pose_param : str
             Param name of the prior for the task
        exit_topic : str
             Topic to listen to and quit out after num_exit_msgs have arrived
        target_classes : str or list
            darknet classes to quit out if they are exist
            if None will use the exit_topic
        target_frames : str or list
            frames to search for the target_classes in
        min_seen_frames : int
            Number of seen frames in occam 0 to quit the search
        darknet_cameras : bool list, len 6
            Darknet cameras to use
        """
        self.listener = tf.TransformListener() # Transform prior into odom
        self.prior_pose_param = prior_pose_param
        self.target_classes = []
        self.target_frames = []
        if target_classes == None:                # Exit by subscribing to a topic
            self.topic_exit = True
            rospy.Subscriber(exit_topic, PoseStamped, self.exit_callback)
        elif type(target_classes) == str:                                   # Exit by a bounding box with our target class being available
            self.topic_exit = False
            self.target_classes.append(target_classes)
            self.service = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
            if type(target_frames) == str:
                self.target_frames.append(target_frames)
            else: # list
                self.target_frames.extend(target_frames)
        else: # target_classes is list
            self.topic_exit = False
            self.target_classes.extend(target_classes)
            self.service = rospy.ServiceProxy('cusub_perception/darknet_multiplexer/get_classes', DarknetClasses)
            if type(target_frames) == str:
                self.target_frames.append(target_frames)
            else: # list
                self.target_frames.extend(target_frames)
            
        self.num_frames_seen = 0
        self.vp_client = actionlib.SimpleActionClient('visual_point', VisualPointAction)
        self.darknet_config = darknet_cameras
        self.min_seen_frames = min_seen_frames
        super(Search, self).__init__(self.outcomes, "Search")
    
    @classmethod
    def from_topic(self, prior_pose_param, exit_topic, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """ Allows the search to quit if a msg is published to exit_topic, default behavior of this class """
        return self(prior_pose_param, exit_topic, darknet_cameras=darknet_cameras)

    @classmethod
    def from_bounding_box(self, prior_pose_param, target_classes, target_frames):
        """ Allows the search to quit if darknet is publishing bounding box msgs of the target class at the current time """
        return self(prior_pose_param, None, target_classes, target_frames)

    def exit_callback(self, msg): # Abort on the first publishing
        if not self.replan_requested():
            rospy.loginfo("...Task Found!")
            self.request_replan()

    def vp_feedback_callback(self, feedback):
        self.num_frames_seen = feedback.target_frame_count

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
        # launch the pointer search        
        goal = VisualPointGoal()
        goal.target_classes = self.target_classes
        goal.target_frame = "leviathan/description/occam0_frame_optical"
        self.vp_client.send_goal(goal, feedback_cb=self.vp_feedback_callback)
        rospy.sleep(3)

        # begin moving to prior
        prior = self.get_odom_prior(self.prior_pose_param)
        self.go_to_pose_non_blocking(prior) 

        while not rospy.is_shutdown():
            if self.num_frames_seen > self.min_seen_frames:
                self.cancel_way_client_goal()
                #probably gonn change later - not the best
                rospy.loginfo("...Found one of %s in one of %s, search success!"%(self.target_classes, self.target_frames))
                self.vp_client.cancel_goal()
                time.sleep(2)
                return "found"
            elif self.num_frames_seen < 0:
                self.cancel_way_client_goal() # todo LB
                self.go_to_pose_non_blocking(prior)
            elif userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                self.vp_client.cancel_goal()
                time.sleep(2)
                return "not_found"
            elif self.check_reached_pose(prior):
                rospy.logerr("Search unable to find task.")
                userdata.outcome = "not_found"
                self.vp_client.cancel_goal()
                time.sleep(2)
                return "not_found"
            rospy.sleep(0.25)

        # Ask the server for available bounding box classes and check if our target_class is among those
        # while not rospy.is_shutdown():
        #     present_classes = self.service(rospy.Duration(1), self.target_frames).classes
            
        #     found = False
        #     for c in self.target_classes:
        #         if c in present_classes:
        #             found = True
            
        #     if found:
        #         break

        #     if userdata.timeout_obj.timed_out:
        #         self.cancel_way_client_goal()
        #         userdata.outcome = "timed_out"
        #         return "not_found"
        #     elif self.check_reached_pose(prior):
        #         rospy.logerr("Search unable to find task.")
        #         userdata.outcome = "not_found"
        #         return "not_found"
        # self.cancel_way_client_goal()
        # return "found"

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

