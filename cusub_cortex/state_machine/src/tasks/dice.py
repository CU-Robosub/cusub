#!/usr/bin/env python

"""
Dice, Attempts to hit the 5 and 6 Dice
Objectives:
1) Search
2) Approach 5
3) Visual Servo 5
4) Backup
5) Approach 6
6) Visual Servo 6
7) Backup
"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from pylab import * # validation of travel equations
from pdb import set_trace

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import random

class Dice(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior, searchAlg):
        super(Dice, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives()
        self.initMapperSubs()
        self.linkObjectives()
    def initObjectives(self):
        pass
    def initMapperSubs(self):
        pass
    def linkObjectives(self):
        pass

class Approach(Objective):
    outcomes = ['success','aborted']
    curPose = None
    
    def __init__(self):
        rospy.loginfo("---Approach objective initializing")
        super(Approach, self).__init__(self.outcomes, "Approach")
        
    def execute(self, userdata):
        rospy.loginfo("Executing Approach")
        # We should have a pose of the 5 dice by now

    def getPath(self, goal_pt, obstacle_list, sub_pt):
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
        # Select 9 points around the dice we're interested in,
        # Get distances of all of the points to all of the other dice
        # Approach point will be in direction with highest distance
        # Path plan w/ square method
        assert isinstance(goal_pt, Point)
        assert isinstance(obstacle_list, list)

        x = goal_pt.x
        y = goal_pt.y
        z = goal_pt.z
        points_around_goal = self.getCirclePoints(x,y, radius=1)
        
#        print(points_around_goal)
        dists=np.zeros((len(points_around_goal),1))
        for obs in obstacle_list:
            vec2 = np.array([obs.x,obs.y], dtype=np.float32)
            print(vec2)
            for i in range(len(points_around_goal)):
                vec1 = np.array(points_around_goal[i], dtype=np.float32)
                diff = vec1 - vec2
                dist = np.linalg.norm(diff)
                if dist <= 1: # heavily weight smaller distances
                    dist -= 20
                dists[i] += dist
        # print(dists)
        # print(np.argmax(dists))
        # print(points_around_goal[np.argmax(dists)])
        approach_pt = points_around_goal[np.argmax(dists)]
        x_app = approach_pt[0]
        y_app = approach_pt[1]

        print("Goal Pt: " + str((x,y)))
        print("App Pt: " + str((x_app, y_app)))

        # goal_pt and obstacle list
        xdata = [i.x for i in obstacle_list]; xdata = [x] + xdata; xdata.append(x_app)
        ydata = [i.y for i in obstacle_list]; ydata = [y] + ydata; ydata.append(y_app)
        zdata = [-5 for i in obstacle_list]; zdata = [-5] + zdata; zdata.append(-5)
        cdata = [0 for i in obstacle_list]; cdata = [1] + cdata; cdata.append(0.5)
        # print(xdata)
        # print(ydata)
        # print(zdata)
        # print(cdata)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter3D(xdata, ydata, zdata, c=np.array(cdata))
        plt.show()
        
    def getCirclePoints(self, x, y, radius=1, num_points_side=21):
        """
        Return a circle of points around an x,y
        Recommend num_points you'd like between -radius and radius + 1
        """
        x_points = np.linspace(-radius,radius,num=num_points_side, endpoint=True)
        sq = np.sqrt( np.power(radius,2) - np.power(x_points, 2) )
        neg_sq = - sq
        y_pos = sq + y
        y_neg = neg_sq + y

        x_points += x

        xdata = list(x_points) + list(x_points) + [x]
        ydata = list(y_pos) + list(y_neg) + [y]
        # comment below
        # zdata = list(np.zeros(len(ydata)))
        # cdata = [0.0 for i in ydata]; cdata[len(ydata)-1] = 10
        # print(xdata)
        # print(len(xdata))
        # print(ydata)
        # print(len(ydata))
        # print(zdata)
        # print(len(zdata))
        # print(cdata)
        # print(len(cdata))

        # fig=plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter3D(xdata, ydata, zdata, c=np.array(cdata))
        # plt.show()
        return zip(xdata,ydata)
        
def main():
    rospy.init_node("dice_node")
    d = Dice()
    rospy.spin()


def genTestPoints(minDist, maxDist):
    """
    Returns a list of 5 points, first is goal point, next 3 other dice points, 5th point the starting position of the sub
    All dice are minDist away and no more than maxDist apart
    Hardcoded between -100 and 100
    """
    goal_pt = Point()    
    goal_pt.x = ( random.random() - 0.5) * 200
    goal_pt.y = ( random.random() - 0.5) * 200
    goal_vec = np.array([goal_pt.x, goal_pt.y], dtype=np.float32)
    print(goal_pt)
    
    
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
    for i in range(10):
        pts = genTestPoints(0.5,1)
#        print(pts)
        
        p = Pose()
        p.position = pts[4]
        a.curPose = p
        
        a.getPath(pts[0], pts[1:4], pts[4])
        
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
    a.getPath(gp, p_list)
    
if __name__ == "__main__":
    try:
        test()
    except rospy.ROSException:
        pass
