"""
Search Objective
"""
from tasks.task import Objective
from geometry_msgs.msg import PoseStamped
import rospy
import tf
from cusub_print.cuprint import bcolors

DARKNET_CAMERAS_DEFAULT=[1,1,1,1,1,0]

class Search(Objective):

    outcomes = ['found','not_found']

    def __init__(self, task_name, listener, target_classes, prior_pose_param_str, darknet_cameras=DARKNET_CAMERAS_DEFAULT):
        """
        Params
        ------
        task_name : str
        listener : DetectionListener
        target_classes : list
        prior_pose_param_str : str
        darknet_cameras : list
        """
        super(Search, self).__init__(self.outcomes, task_name + "/Search")
        self.darknet_config = darknet_cameras
        self.prior_pose_param_str = prior_pose_param_str
        self.target_classes = target_classes
        self.det_listener = listener
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        self.cuprint("executing")
        self.configure_darknet_cameras(self.darknet_config)
        prior = self.get_odom_prior(self.prior_pose_param_str) # attempt to grab from mapper first --> we may already have localized it
        
        self.toggle_waypoint_control(False)
        self.go_to_pose_non_blocking(prior)

        classes_colored = [bcolors.HEADER + x + bcolors.ENDC for x in self.target_classes]
        self.cuprint("approaching prior, listening for detections of " + ", ".join(classes_colored))
        while not rospy.is_shutdown():
            # Check for detection
            if self.query_listener(): 
                self.cuprint("detected class")
                userdata.outcome = "success"
                self.cancel_way_client_goal()
                return "found"
            # Check for timeout
            elif userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"
            # Check for reaching the prior pose
            elif self.check_reached_pose(prior):
                self.cuprint("reached prior without any detections", warn=True)
                self.cuprint("spiral search functionality not implemented...quitting", warn=True)
                self.cancel_way_client_goal()
                userdata.outcome = "not_found"
                return "not_found"

            rospy.sleep(0.25)

        return "found"

    # Checks listener for existence of any of self.target_classes
    def query_listener(self):
        dobj_dict = self.det_listener.query_classes(self.target_classes)
        if not dobj_dict: # No classes present
            return False
        else:             # At least one class is present, check if there's been a new dvector
                          # We take this extra step for the unlikely case we return to Search from another state
            for key in dobj_dict.keys():
                dobj_nums = dobj_dict[key]
                for num in dobj_nums:
                    if self.det_listener.check_new_dv(num):
                        return True
        return False
        
    def get_odom_prior(self, rosparam_str):
        if not rospy.has_param(rosparam_str):
            raise(Exception("Could not locate rosparam: " + rosparam_str))
            
        xyzframe_list = rospy.get_param(rosparam_str)
        p = PoseStamped()
        p.pose.position.x = xyzframe_list[0]
        p.pose.position.y = xyzframe_list[1]
        p.pose.position.z = xyzframe_list[2]
        if len(xyzframe_list) == 4:     # prior needs to be transformed
            self.cuprint("...transforming prior pose to odom")
            p.header.frame_id = xyzframe_list[3]
            p.header.stamp = rospy.Time()
            if not rospy.has_param("~robotname"):
                raise("Launch file must specify private param 'robotname'")
            try:
                odom_frame = '/'+ rospy.get_param("~robotname") + '/description/odom'
                self.cuprint("...waiting for transform: " + odom_frame + " -> /" + xyzframe_list[3])
                self.tf_listener.waitForTransform(p.header.frame_id, odom_frame, p.header.stamp, rospy.Duration(5))
                self.cuprint("...found transform")
                p = self.tf_listener.transformPose(odom_frame, p)
            except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
                rospy.logerr(e)
        return p.pose

