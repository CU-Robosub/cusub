#!/usr/bin/env python
from __future__ import division
"""
StartGate Task, attempts to go through the start gate
Objectives:
1) Search (based on prior)
2) Attack (goes behind gate based on arg distBehindGate)

"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Bool, Float64
import numpy as np
from waypoint_navigator.srv import ToggleControl
import tf

class StartGate(Task):
    name = "start_gate"
    is_three_leg = False

    def __init__(self):
        super(StartGate, self).__init__() # become a state machine first
        self.init_objectives()
        self.link_objectives()

    def init_objectives(self):
        self.search = Search(self.get_prior_param(), "cusub_cortex/mapper_out/start_gate")
        self.attack = Attack()

    def link_objectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('Search', self.search, transitions={'found':'Attack', 'not_found':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})
            smach.StateMachine.add('Attack', self.attack, transitions={'success':'manager','timed_out':'manager'}, \
                remapping={'timeout_obj':'timeout_obj', 'outcome':'outcome'})

class Attack(Objective):
    """
    Tell the sub to go through the gate
    """

    outcomes=['success', 'timed_out']

    def __init__(self):
        rospy.loginfo("Loading attack")
        super(Attack, self).__init__(self.outcomes, "Attack")
        self.dist_behind = rospy.get_param('tasks/start_gate/dist_behind_gate', 1.0)
        self.replan_threshold = rospy.get_param('tasks/start_gate/replan_thresh', 1.0)
        self.leg_adjustment_meters = rospy.get_param('tasks/start_gate/third_leg_adjustment', 0.5)
        self.style_dist = rospy.get_param('tasks/start_gate/style_dist', 0.5)
        self.three_leg_dist = rospy.get_param('tasks/start_gate/three_leg_dist', 1.5)
        self.three_leg_depth = rospy.get_param('tasks/start_gate/three_leg_depth', 1.0)
        self.do_style = rospy.get_param('tasks/start_gate/do_style', False)
        self.spin_carrot = rospy.get_param('tasks/start_gate/spin_carrot_rads', 0.3)
        self.first_pose_received = False
        self.start_gate_pose = None
        self.small_leg_left_side = None
        self.started = False
        self.current_yaw = None
        self.current_depth = None
        self.yaw_pub = rospy.Publisher("cusub_common/motor_controllers/pid/yaw/setpoint", Float64, queue_size=10)
        rospy.loginfo("...waiting for cusub_common/toggleWaypointControl")
        rospy.wait_for_service("cusub_common/toggleWaypointControl")
        rospy.loginfo("...found service")
        rospy.Subscriber("cusub_cortex/mapper_out/start_gate", PoseStamped, self.start_gate_callback)
        rospy.Subscriber("cusub_perception/start_gate/small_pole_left_side", Bool, self.small_leg_callback)
        rospy.Subscriber("cusub_common/motor_controllers/pid/yaw/state", Float64, self.yaw_callback)
        # add depth knowledge
        self.depth_pub = rospy.Publisher("cusub_common/motor_controllers/pid/depth/setpoint", Float64, queue_size=1)
        self.depth_sub = rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depth_callback)

    def yaw_callback(self, msg):
        self.current_yaw = msg.data

    def depth_callback(self,msg):
        self.current_depth = msg.data

    def small_leg_callback(self, msg):
        rospy.loginfo_throttle(0.0001, "Received third leg side.")
        if self.started and self.small_leg_left_side == None: # Receiving leg for the first time
            self.request_replan()
        elif self.started and self.small_leg_left_side != msg.data: # leg has switched sides!
            rospy.logwarn("Third leg has switched sides!")
            self.request_replan()
        self.small_leg_left_side = msg.data

    def start_gate_callback(self, msg):
        # Set the first pose and don't abort the Attack Objective
        if self.start_gate_pose == None:
            self.start_gate_pose = msg
            return

        change_in_pose = self.get_distance(self.start_gate_pose.pose.position, msg.pose.position)

        if change_in_pose > self.replan_threshold:
            self.start_gate_pose = msg
            self.request_replan() # this will loop us back to execute

    def do_gate_with_style(self, userdata):
        while not rospy.is_shutdown():          # Loop over the replans from a gate pose change
            target_pose = self.adjust_gate_pose(
                self.cur_pose, \
                self.start_gate_pose.pose, \
                self.dist_behind, \
                self.small_leg_left_side, \
                self.leg_adjustment_meters)

            dist_in_front_of_gate = self.style_dist + self.dist_behind
            style_pose = self.get_style_pose(self.cur_pose, target_pose, dist_in_front_of_gate)

            if self.go_to_pose(style_pose, userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "timed_out"
                else: # Replan has been requested loop again
                    pass
            else: # Pose reached successfully!
                self.is_three_leg = True
                break

        rospy.loginfo("...reached style pose")
        self.enact_style()
        target_pose.position.z = self.three_leg_depth
        if self.go_to_pose(target_pose, userdata.timeout_obj, replan_enabled=False):
            userdata.outcome = "timed_out"
            return "timed_out"
        else:
            userdata.outcome = "success"
            return "success"

    def do_gate_no_style(self, userdata):
        while not rospy.is_shutdown():          # Loop over the replans from a gate pose change
            target_pose = self.adjust_gate_pose(
                self.cur_pose, \
                self.start_gate_pose.pose, \
                self.dist_behind, \
                self.small_leg_left_side, \
                self.leg_adjustment_meters)

            #three leg pose is the same concept as style pose. Reusing funciton
            dist_in_front_of_gate = self.three_leg_dist + self.dist_behind
            three_leg_pose = self.get_style_pose(self.cur_pose, target_pose, dist_in_front_of_gate)

            if self.go_to_pose(three_leg_pose, userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "timed_out"
            else:
                self.is_three_leg = True

            target_pose.position.z = self.three_leg_depth

            if self.go_to_pose(target_pose, userdata.timeout_obj):
                if userdata.timeout_obj.timed_out:
                    userdata.outcome = "timed_out"
                    return "timed_out"
                else: # Replan has been requested loop again
                    pass
            else:
                userdata.outcome = "success"
                return "success"

    def execute(self, userdata):
        self.started = True
        rospy.loginfo("Executing Attack")
        self.configure_darknet_cameras([1,1,0,0,1,0])
        if self.do_style:
            return self.do_gate_with_style(userdata)
        else:
            return self.do_gate_no_style(userdata)
    #
    # def go_thru_small_side(self):
    #     '''
    #     Responsibilities:
    #         - adjust pose using third leg (should be taken care of by is_three_leg flag)
    #         - set depth to proper value
    #         - strafe to line up with new pose
    #         - go_to_target
    #     '''
    #     # update startgate pose
    #     target_pose = self.adjust_gate_pose(
    #             self.cur_pose, \
    #             self.start_gate_pose.pose, \
    #             self.dist_behind, \
    #             self.small_leg_left_side, \
    #             self.leg_adjustment_meters)
    #
    #     # TODO: figure out how to line up

        #depth_pub.publish()

    def enact_style(self):
        if self.current_yaw == None:
            rospy.logwarn("Start gate never received yaw state!")
            return
        try:
            toggle_control = rospy.ServiceProxy("cusub_common/toggleWaypointControl", ToggleControl)
            res = toggle_control(False)

            radians_turned = 0
            yaw_set = Float64()
            yaw_set.data = self.current_yaw
            while radians_turned < 4*np.pi and not rospy.is_shutdown():
                prev_set = yaw_set.data
                yaw_set.data = self.current_yaw + self.spin_carrot
                yaw_set_diff = yaw_set.data - prev_set
                if abs(yaw_set_diff) < np.pi: # check for wrapping
                    radians_turned += yaw_set_diff
                self.yaw_pub.publish(yaw_set)
            toggle_control(True)
        except rospy.ServiceException, e:
            rospy.logerr(e)

    @staticmethod
    def get_style_pose(cur_pose, goal_pose, style_dist):
        """
        Gets the pose to do style points in front of the gate
        """
        # Find line from sub to buoy
        if ( round(cur_pose.position.x, 2) == round(goal_pose.position.x,2) ): # Avoid infinite slope in the polyfit
            goal_pose.position.x += 0.1
        if ( round(cur_pose.position.y, 2) == round(goal_pose.position.y,2) ): # Avoid infinite slope in the polyfit
            goal_pose.position.y -= 0.1

        x_new = goal_pose.position.x
        y_new = goal_pose.position.y

        # Adjust buoy pose behind the buoy
        x2 = np.array([cur_pose.position.x, x_new])
        y2 = np.array([cur_pose.position.y, y_new])
        m2, b2 = np.polyfit(x2,y2,1)
        m2 = round(m2, 2)
        b2 = round(b2, 2)
        x_hat2 = np.sqrt( ( style_dist**2) / (m2**2 + 1) )
        if x_new > cur_pose.position.x:
            x_hat2 = -x_hat2
        x_new2 = x_new + x_hat2
        y_new2 = x_new2 * m2 + b2

        # Find target yaw
        dx = goal_pose.position.x - x_new2
        dy = goal_pose.position.y - y_new2
        target_yaw = np.arctan2(dy, dx)
        quat_list = tf.transformations.quaternion_from_euler(0,0, target_yaw)

        # Make Pose Msg
        target_pose = Pose()
        target_pose.position.x = x_new2
        target_pose.position.y = y_new2
        target_pose.position.z = goal_pose.position.z
        target_pose.orientation.x = quat_list[0]
        target_pose.orientation.y = quat_list[1]
        target_pose.orientation.z = quat_list[2]
        target_pose.orientation.w = quat_list[3]

        return target_pose

    @staticmethod
    def adjust_gate_pose(cur_pose, gate_pose, dist_behind, small_leg_left_side, third_leg_adjustment):
        """
        Draws a line from the sub to the gate and adjusts the gate's pose along this line, behind the gate, by
        'dist_behind'. Likewise, if small_leg_left_side is not None, a perpendicular line to the first is created
        and according to whichever side the small leg is on, the gate's pose is adjusted laterally by 'third_leg_adjustment'
        ( Assumes gate is approximately facing the sub. )

        Parameters
        ----------
        cur_pose : Pose
            current pose of the sub
        gate_pose : Pose
        dist_behind : float
        small_leg_left_side : bool
        third_leg_adjustment : float

        Returns
        -------
        target_pose : Pose
        """
        # Find line from sub to gate
        if ( round(cur_pose.position.x, 2) == round(gate_pose.position.x,2) ): # Avoid infinite slope in the polyfit
            gate_pose.position.x += 0.1
        if ( round(cur_pose.position.y, 2) == round(gate_pose.position.y,2) ): # Avoid infinite slope in the polyfit
            gate_pose.position.y -= 0.1

        if not self.is_three_leg:
            return;
        else:
            # Adjust the gate pose to go through small leg side
            if small_leg_left_side != None:
                x = np.array([cur_pose.position.x, gate_pose.position.x])
                y = np.array([cur_pose.position.y, gate_pose.position.y])
                m, b = np.polyfit(x,y,1)
                if m == 0:
                    m = 0.00001
                perp_m = - 1 / m
                perp_b = gate_pose.position.y - perp_m * gate_pose.position.x
                x_hat = np.sqrt( ( third_leg_adjustment**2) / (perp_m**2 + 1) )
                if (gate_pose.position.y > cur_pose.position.y) and small_leg_left_side:
                    x_hat = - x_hat
                elif (gate_pose.position.y < cur_pose.position.y) and not small_leg_left_side:
                    x_hat = - x_hat
                x_new = gate_pose.position.x + x_hat
                y_new = x_new*perp_m + perp_b
            else:
                rospy.logwarn("Start Gate SM didn't receive 3rd leg side")
                x_new = gate_pose.position.x
                y_new = gate_pose.position.y

        # Adjust gate pose behind the gate
        x2 = np.array([cur_pose.position.x, x_new])
        y2 = np.array([cur_pose.position.y, y_new])
        m2, b2 = np.polyfit(x2,y2,1)
        m2 = round(m2, 2)
        b2 = round(b2, 2)
        x_hat2 = np.sqrt( ( dist_behind**2) / (m2**2 + 1) )
        if x_new <= cur_pose.position.x:
            x_hat2 = -x_hat2
        x_new2 = x_new + x_hat2
        y_new2 = x_new2 * m2 + b2

        target_pose = Pose()
        target_pose.position.x = x_new2
        target_pose.position.y = y_new2
        target_pose.position.z = gate_pose.position.z
        return target_pose
