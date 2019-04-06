#!/usr/bin/python2
from __future__ import division
"""
Dice, Attempts to hit the 5 and 6 Dice
Objectives:
1) Search
---
Loop for all targets
2) Approach the first target in strafe mode
3) Visual Servo
4) Backup

"""
from __future__ import division
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from pylab import * # validation of travel equations
from pdb import set_trace
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import random
from tasks.graph import graphGetPath
from tasks.naive_servo import NaiveVisualServoTool, ServoAxisConfig
import tf
from sensor_msgs.msg import Imu

APPROACH_TIMEOUT = 3

class Dice(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self):
        super(Dice, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives()
        self.linkObjectives()
        
    def initObjectives(self):
        self.targets = rospy.get_param('tasks/dice/targets')
        search_alg = rospy.get_param('tasks/dice/search_alg')
        prior = self.priorList2Pose(rospy.get_param('tasks/dice/prior'))
        if not self.targets:
            raise Exception("No targets given to Dice.")
        
        self.search = Search(search_alg, prior, 'cusub_cortex/mapper_out/'+self.targets[0])
        self.approaches = []
        self.attacks = []
        for t in self.targets:
            self.approaches.append(Approach(t))
            self.attacks.append(Attack(t))
            
    def linkObjectives(self):
        with self:
            name_first_app = "Approach_" + self.targets[0]
            smach.StateMachine.add("Search", self.search, transitions={'aborted':name_first_app, 'success':'Search'})
            for i in range(len(self.targets)):
                name_app = "Approach_" + self.targets[i]
                name_att = "Attack_" + self.targets[i]
                smach.StateMachine.add(name_app, self.approaches[i], transitions={"success":name_att, "aborted":"task_aborted", "replan":name_app})
                
                if i+1 != len(self.targets): # we have more targets to go
                    name_app_next = "Approach_" +self.targets[i+1]
                    smach.StateMachine.add(name_att, self.attacks[i], transitions={"success":name_app_next, "aborted":"task_aborted"})
                else: # final attack
                    smach.StateMachine.add(name_att,self.attacks[i], transitions={"success":"task_success", "aborted":"task_aborted"})
            
class Approach(Objective):
    # Needs all poses
    outcomes = ['success','aborted', 'replan']
    curPose = None
    object_poses = {}
    
    def __init__(self, target):
        
        rospy.loginfo("---Approach " + target +" objective initializing")
        super(Approach, self).__init__(self.outcomes, "Approach_" + target)
        self.load_rosparams()
        self.target = target
        self._replan_requested = False
        
        rospy.Subscriber('cusub_cortex/mapper_out/dice1', PoseStamped, self.callback)
        rospy.Subscriber('cusub_cortex/mapper_out/dice2', PoseStamped, self.callback)
        rospy.Subscriber('cusub_cortex/mapper_out/dice5', PoseStamped, self.callback)
        rospy.Subscriber('cusub_cortex/mapper_out/dice6', PoseStamped, self.callback)

    def load_rosparams(self):
        self.replan_thresh = float(rospy.get_param('tasks/dice/replan_threshold'))
        self.orbital_radius = float(rospy.get_param('tasks/dice/orbital_radius'))
        self.approach_radius = float(rospy.get_param('tasks/dice/approach_radius'))
        assert(self.orbital_radius > self.approach_radius)
        
        
    def clear_replan(self):
        self._replan_requested = False
        self.clear_abort()
        
    def request_replan(self):
        rospy.loginfo("---replanning")
        self._replan_requested = True
        self.request_abort()
        
    def replan_requested(self):
        return self._replan_requested

    def callback(self, msg):
        topic = msg._connection_header['topic']
        obj_name = topic.split('/')[-1]

        if obj_name not in self.object_poses.keys():
            self.object_poses[obj_name] = msg
            if self.started:
                self.request_replan()
            return
        prev_pose = self.object_poses[obj_name]
        self.object_poses[obj_name] = msg    
        if self.getDistance(prev_pose.pose.position, msg.pose.position) > self.replan_thresh:
            if self.started:
                self.request_replan()
        
    def execute(self, userdata):
        rospy.loginfo("---Executing Approach for " + self.target)
        if self.replan_requested():
            self.clear_replan()

        rospy.wait_for_message("cusub_cortex/mapper_out/" + self.target, PoseStamped, APPROACH_TIMEOUT)

        if self.target not in self.object_poses.keys(): # We haven't received the object's pose yet, NOTE passing the search objective just means we have the first target's pose, possibly not subsequent targets
            rospy.logwarn("Haven't received: " + self.target + "'s pose. Aborting")
            return "aborted"

        self.started = True
        dice_list = [die.pose.position for key,die in self.object_poses.iteritems()]
        # dice_list = [value.position for key,value in self.object_poses.items()] # Python3        
        path = self.getApproachPath(self.object_poses[self.target].pose.position, dice_list)
        
        pose = path.pop(0)
        
        status = self.goToPose(pose, useYaw=True)
        if status:
            if self.replan_requested():
                self.clear_replan()
                return "replan"
            else:
                return "aborted"
        
        for goal in path:
            status = self.goToPose(goal, useYaw=False)
            if status:
                if self.replan_requested():
                    self.clear_replan()
                    return "replan"
                else:
                    return "aborted"

        # Face the dice
        # goal_pose = Pose()        
        # quat = self.getFacingQuaternion(self.curPose.position, self.object_poses[self.target].pose.position)
        # goal_pose.orientation = quat
        # goal_pose.position = self.curPose.position
        # self.goToPose(goal_pose, useYaw=True)
        # TODO we may need to add a some sort of depth preparation before visual servoing
        rospy.sleep(2)
        return "success"

    def getApproachPath(self, goal_pt, dice_list):
        """
        Returns a list of points to go to, to reach the desired goal point
        Uses a surrounding box as helper points along its way to the approach point
        The D's are dice, and the S is the sub, and the A is the approach point. X's are easy to calculate stops we could use to reach the approach point. Our algorithm will find the X*'s to use as waypoints along our path to the approach point.
        Path Plannign for Squares:
        X      A X*
            D  D
          D  D
        X    S   X*

        Points around dice:
        X X X
        X D X
        X X X
        """
        # Get distances of all of the points to all of the other dice
        # Approach point will be in direction with highest distance
        assert isinstance(goal_pt, Point)
        assert isinstance(dice_list, list)

        x = goal_pt.x
        y = goal_pt.y
        z = goal_pt.z
        points_around_goal = self.getCirclePoints(x,y,z, radius=self.approach_radius)

        # Find the takeoff point
        dists=np.zeros((len(points_around_goal),1))
        for die in dice_list:
            vec2 = np.array([die.x,die.y], dtype=np.float32)
            # print(vec2)
            for i in range(len(points_around_goal)):
                new_list = [points_around_goal[i].x, points_around_goal[i].y]
                vec1 = np.array(new_list, dtype=np.float32)
                diff = vec1 - vec2
                dist = np.linalg.norm(diff)
                if dist <= self.approach_radius: # heavily weight smaller distances
                    dist -= 20
                dists[i] += dist
        # print(dists)
        # print(np.argmax(dists))
        # print(points_around_goal[np.argmax(dists)])
        takeoff_pt = points_around_goal[np.argmax(dists)]
        x_takeoff = takeoff_pt.x
        y_takeoff = takeoff_pt.y
        # print("Goal Pt: " + str((x,y)))
        # print("Takeoff Pt: " + str((x_takeoff, y_takeoff)))

        centroid = self.getCentroid(dice_list + [goal_pt])
        horizon_pts = self.getCirclePoints(centroid.x, centroid.y, z, self.orbital_radius, 6)
        path_pts = graphGetPath(takeoff_pt, self.curPose.position, horizon_pts)
        path_poses = self.points2Poses(goal_pt, path_pts)

        # goal_pt and dice list
        # xdata = [i.x for i in dice_list]; xdata = [x] + xdata;  xdata += [i.x for i in path]; 
        # ydata = [i.y for i in dice_list]; ydata = [y] + ydata;  ydata += [i.y for i in path]; 
        # zdata = [-5 for i in dice_list]; zdata = [-5] + zdata;  zdata += [-5 for i in path]; 
        # cdata = [0 for i in dice_list]; cdata = [1] + cdata;  cdata += [0.7 for i in path];
        
        # xdata.append(centroid.x)
        # ydata.append(centroid.y);
        # zdata.append(-5);
        # cdata.append(0.5);
        # xdata.append(self.curPose.position.x)
        # ydata.append(self.curPose.position.y)
        # zdata.append(-5)
        # cdata.append(2)
        # print(xdata)
        # print(ydata)
        # print(zdata)
        # print(cdata)
        
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # # ax.scatter3D(xdata, ydata, zdata, c=np.array(cdata))
        # ax.scatter(xdata, ydata, c=np.array(cdata))
        # plt.show()

        return path_poses

    def points2Poses(self, target_pt, pt_list):
        pose_list = []
        next_quat = self.getFacingQuaternion(target_pt, pt_list[0])
        for pt in pt_list:
            pose = Pose()
            pose.position = pt
            pose.orientation = next_quat
            next_quat = self.getFacingQuaternion(target_pt, pt)
            pose_list.append(pose)
        return pose_list

    def getFacingQuaternion(self, target_pt, current_pt):
        roll, pitch = 0, 0
        x_diff = target_pt.x - current_pt.x
        y_diff = target_pt.y - current_pt.y
        yaw = np.arctan2(y_diff, x_diff)
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        quat = Quaternion()
        quat.x = quat_list[0]
        quat.y = quat_list[1]
        quat.z = quat_list[2]
        quat.w = quat_list[3]        
        return quat
    
    def getCentroid(self, pts):
        x_sum = 0
        y_sum = 0
        for p in pts:
            x_sum += p.x
            y_sum += p.y
        avg_pt = Point()
        avg_pt.x = x_sum / len(pts)
        avg_pt.y = y_sum / len(pts)
        return avg_pt
        
        
    def getCirclePoints(self, x, y, z, radius=1, num_points=20):
        """
        Return a circle of points around an x,y at depth z
        Recommend num_points you'd like between -radius and radius + 1
        """
        angles = np.linspace(0, 2*np.pi, num=num_points, endpoint=False)
        x_coords = radius * np.cos(angles) + x
        y_coords = radius * np.sin(angles) + y

        pts = []
        for j in range(len(x_coords)):
            new_pt = Point()
            new_pt.x = x_coords[j]
            new_pt.y = y_coords[j]
            new_pt.z = z
            pts.append(new_pt)
                
        
        # xdata = list(x_coords) + [x]
        # ydata = list(y_coords) + [y]
        # zdata = list(np.zeros(len(ydata)))
        # cdata = [0.0 for i in ydata]; cdata[len(ydata)-1] = 10
        # fig=plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter3D(xdata, ydata, zdata, c=np.array(cdata))
        # plt.show()

        # Turn the data into points
        return pts


class Attack(Objective):
    outcomes = ['success','aborted']
    
    def __init__(self, target):
        rospy.loginfo("---Attack " + target + " Initializing")
        super(Attack, self).__init__(self.outcomes, "Attack")
        param_dict = self.load_rosparams() # TODO put all configs into the load rosparams method
        self.active = False
        self.target = target
        sac = ServoAxisConfig('yaw', 'occam0', 376, False, 100)
        self.servo_tool = NaiveVisualServoTool(self.target, self.handle_servoing, sac, lockon_time=param_dict['lockon_time'])
        rospy.Subscriber('cusub_common/motor_controllers/pid/drive/state', Float64, self.drive_callback)
        self.drive_pub = rospy.Publisher('cusub_common/motor_controllers/pid/drive/setpoint', Float64, queue_size=1)
        rospy.Subscriber("cusub_common/imu", Imu, self.imu_callback)
        self.spike = False
        self.timeout = float(rospy.get_param("tasks/dice/attack_timeout"))

    def load_rosparams(self):
        param_dict = {} # for things we don't need to store permanently
        self.accel_spike_thresh = float(rospy.get_param('tasks/dice/accel_spike_thresh', '-0.15'))
        self.accel_axis = rospy.get_param('tasks/dice/accel_axis')
        self.carrot_dist = float(rospy.get_param('tasks/dice/carrot_dist'))
        self.backup_dist = float(rospy.get_param('tasks/dice/backup_dist'))
        param_dict['lockon_time'] = float(rospy.get_param('tasks/dice/lockon_time', '5.0'))
        return param_dict
    
    def complete_servoing(self):
        self.spike = True
        self.active = False
        self.servo_tool.active = False
        self.timer.shutdown()

    def timeout_callback(self, msg):
        self.timer.shutdown()
        rospy.loginfo("---Attack " + self.target + " timed out. Triggering spike")
        self.complete_servoing()

    def drive_callback(self, msg):
        self.current_drive = msg.data
        
    def handle_servoing(self, image, box):
        drive_msg = Float64()
        if self.spike:
            rospy.loginfo("Spike!")
            drive_msg.data = self.current_drive # stop
            self.drive_pub.publish(drive_msg)
            self.servo_tool.deactivate()
        else:
            drive_msg.data = self.current_drive + self.carrot_dist
            self.drive_pub.publish(drive_msg)
            
    def imu_callback(self, msg):
        accel = msg.linear_acceleration.x # default to x

        if self.accel_axis == 'x':
            accel = msg.linear_acceleration.x
        elif self.accel_axis == 'y':
            accel = msg.linear_acceleration.y
        elif self.accel_axis == 'z':
            accel = msg.linear_acceleration.z
        else:
            rospy.logwarn("unrecognized acceleration axis in imu callback")
        
        if accel < self.accel_spike_thresh and self.active:
            self.complete_servoing()

    def get_pose_behind(self, current_pose, dist_behind):

        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z
        
        quat = current_pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        backwards_yaw = yaw + np.pi
        
        new_x = dist_behind * np.cos(backwards_yaw) + x
        new_y = dist_behind * np.sin(backwards_yaw) + y

        new_pose = Pose()
        new_pose.position.x = new_x
        new_pose.position.y = new_y
        new_pose.position.z = z
        new_pose.orientation = quat
        return new_pose
        
    def backup(self):
        rospy.loginfo("---backing up")

        new_pose = self.get_pose_behind(self.curPose, self.backup_dist)
        # rospy.loginfo("Going from pose:")
        # rospy.loginfo(self.curPose)
        # rospy.loginfo("To pose:")
        # rospy.loginfo(new_pose)
        self.goToPose(new_pose, useYaw=False)
    
        # msg = Float64()
        # msg.data = self.current_drive - self.backup_dist
        # self.drive_pub.publish(msg)

        # while abs(self.current_drive - msg.data) > 0.5 and not rospy.is_shutdown():
        #     rospy.sleep(0.25)
        
    def execute(self, userdata):
    
        self.active = True
        rospy.loginfo("---Executing Attack for " + self.target)
        
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.timeout), self.timeout_callback)

        self.servo_tool.run()

        self.backup()
        return "success"
        
def genTestPoints(minDist, maxDist):
    """
    Returns a list of 5 points
    Index0: goal point
    Indices1-3: dice points
    Index4: starting position of the sub

    All dice are minDist away and no more than maxDist apart
    """

    # Should be some point randomly -100 through 100
    goal_pt = Point()    
    goal_pt.x = ( random.random() - 0.5) * 200
    goal_pt.y = ( random.random() - 0.5) * 200
    goal_vec = np.array([goal_pt.x, goal_pt.y], dtype=np.float32)
    
    dice = []
    while len(dice) < 3:
        x = (random.random() - 0.5) * (maxDist / 0.5) + goal_pt.x
        y = (random.random() - 0.5) * (maxDist / 0.5) + goal_pt.y
        vec = np.array([x,y], dtype=np.float32)

        diff = vec - goal_vec
        if np.linalg.norm(diff) < minDist:
            continue
        
        goodPos = True
        for i in dice:
            diff = vec - np.array([i.x,i.y], dtype=np.float32)
            if np.linalg.norm(diff) < minDist:
                goodPos = False
                break
        if goodPos:
            p = Point()
            p.x = x
            p.y = y
            dice.append(p)
            
    start_pt = Point()            
    start_pt.x = ( random.random() - 0.5) * 200
    start_pt.y = ( random.random() - 0.5) * 200

    # point_list = [goal_pt] + dice # + [start_pt]
    # xdata = [i.x for i in point_list]
    # ydata = [i.y for i in point_list]
    # cdata = np.zeros((len(ydata))) + 5
    # cdata[0] = 10.0

    # fig=plt.figure()
    # ax = fig.add_subplot(111)
    # ax.scatter(xdata, ydata, cdata, cmap='Greens')
    # plt.show()

    return [goal_pt] + dice + [start_pt]


def test():
    """
    Basically we have 4 dice and we want to get a path to an approach point
    Returns list of points to travel to in order, last point is the approach pt
    """
    rospy.init_node("dice_node")
    a = Approach()
    
    for i in range(1):
        pts = genTestPoints(0.5,1) # min dist & max dist
        # print(pts)
        
        p = Pose()
        p.position = pts[4]
        a.curPose = p
        
        a.getApproachPath(pts[0], pts[1:4])
        
    return
    # for i in range(10):
    #     genTestPoints(0.3, 1)
    # a.getCirclePoints(0,0, 3, 21)
    # return
    
    gp = Point(); gp.x = 0; gp.y = 0;
    # p1 = Point(); p1.x = -1; p1.y = 0;
    # p2 = Point(); p2.x = 0; p2.y = -1;
    # p3 = Point(); p3.x = 0; p3.y = 1;
    p1 = Point(); p1.x = -1; p1.y = 0;
    p2 = Point(); p2.x = -1; p2.y = -1;
    p3 = Point(); p3.x = -1; p3.y = 1;
    p_list = [p1,p2,p3]
    a.getApproachPath(gp, p_list)

                      
if __name__ == "__main__":
    try:
        test()
    except rospy.ROSException:
        pass
