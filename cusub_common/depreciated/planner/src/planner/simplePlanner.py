#!/usr/bin/env python

"""
Simple planner, given an xyzw current pose and xyzw goal pose, will calculate a movement in drive, strafe, depth and possible multiple headings

"""
import numpy as np
import rospy

"""
 is positive strafe to the right or left of the sub??
 how does the heading function? -> ie does it go over 360 degrees?
     Is it equivalent to global as in I can just say turn to -90 degrees globally?


CHANGE TO POS IS DOWN!!!

"""

class SimplePlanner():

    def __init__(self, currentPose):
        self.curPose = currentPose # should be EulerPose type
        
    def setCurrentPose(self, currentPose):
        self.curPose = currentPose
    
    def getRelMovementStrafe(self, goalPose):
        if not isinstance(goalPose, EulerPose):
            rospy.logerr("Incorrent arguments, provide an EulerPose")
            return
        
        x0 = self.curPose.x
        y0 = self.curPose.y

        # NOTE, DRAW IT OUT AND FIGURE OUT WHICH DIRECTION I'M MESSING UP

        x1 = goalPose.x
        y1 = goalPose.y

        # calculate the initial b term
        m1 = np.tan(goalPose.w) # calculate our slope
        m0 = - ( 1 / m1 )
        b0 = y0 - m0 * x0

        # calculate the final b term
        b1 = y1 - m1 * x1
#        print(b1)

        # solve for the final strafe point
        mat = np.matrix([ [ -m0, 1 ], [ -m1, 1 ] ])
        print(mat)
        matInv = np.linalg.inv(mat)
#        print('#')
#        print(matInv)
        y = np.matrix([ [b0],[b1] ])
#        print(y)
#        print('##')
        strafeMat = np.multiply(matInv, y)
#        print(strafeMat)

        strafePt = strafeMat[1]
#        print(strafePt)

        # we know the strafe point, let's find the strafe distance
        diff_x = strafePt.item(0) - self.curPose.x
        diff_y = strafePt.item(1) - self.curPose.y
#        print(diff_x)
#        print(diff_y)

        strafeDist = np.sqrt( diff_x**2 + diff_y**2 )
        # determine in which direction we need to strafe (left or right)
        strafeSide = m0 * strafePt.item(0) + strafePt.item(1) - b0
        print(strafeSide)
        if strafeSide > 0:
            print("strafing left")
            strafeDist = np.negative(strafeDist) # why can't I just put a minus infront of this...
            print(strafeDist)

        # now let's find the drive distance
        diff_x2 = goalPose.x - strafePt.item(0)
        diff_y2 = goalPose.y - strafePt.item(1)
        driveDist = np.sqrt( diff_x2**2 + diff_y2**2 )

        # determine drive forwards or backwards...
        driveSide = m1 * x1 + y1 - b1
        print(driveSide)
        if driveSide > 0:
            print("driving backwards") # this is probably not good...
            driveDist = np.negative(driveDist)

        return [goalPose.w, strafeDist, driveDist]        
    
    def getRelMovementNoStrafe(self, goalPose, curPose):
        # NOTE THE DEGREES ARE NOT A CHANGE BUT THE ACTUAL DEGREES THE SUB SHOULD TURN TOWARD (whereas drive and strafe are differences from the current state ie drive should increase by __ and strafe should increase by __)
        
        if not isinstance(goalPose, EulerPose) or not isinstance(curPose, EulerPose):
            rospy.logerr("Incorrent arguments, provide an EulerPose")
            return
        
        diff_x = goalPose.x - curPose.x
        diff_y = goalPose.y - curPose.y
        diff_z = goalPose.z - curPose.z

        movement=[]
        movement.append(diff_z)

        h = np.sqrt( diff_x**2 + diff_y**2 )
        
        if h == 0:
            print("Already @ the point")
            return [goalPose.w, None, None, None]

        theta = np.arccos( diff_x / h )
        if diff_y < 0:
            theta = -theta

        movement.append(theta)

        # add in zero strafe
        movement.append(None)

        # append the dist
        movement.append(h)

        # append final orientation
        movement.append(goalPose.w)

        return movement
        
"""
Reasons for a custom class pose
- accessing elements is quite easy pose.x ... pose.roll making debugging easier
- storing an euler pose vs an odometry message, we get euler coords which are easier to understan
- we don't need to go through several message types to access what we want ie no pose.pose.pose.position.x
"""

class EulerPose():
    """
    A simple pose object that serves as an alternative to an array
    """
    def __init__(self, x, y, z, yaw, pitch=0.0, roll=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = yaw
        self.p = pitch
        self.r = roll
        

def testRelMoveStrafe():
    print("initial pose: 0,0,0,90degrees")
    curPose = EulerPose(0,0,0,0)
    s = SimplePlanner(curPose)

    print("4,1,-3, -45 degrees")
    goalPose = EulerPose(4,1,-3, -np.pi / 4)
    print(s.getRelMovementStrafe(goalPose))

    print("\n##################\n")
    
    print("1,1,-3, 180 degrees")
    goalPose = EulerPose(1,1,-3,np.pi)
    print(s.getRelMovementStrafe(goalPose))

    print("\n#################\n")

    # more different kinds of tests...
    print("1,1,-3, -180 degrees")
    goalPose = EulerPose(1,1,-3,-np.pi)
    print(s.getRelMovementStrafe(goalPose))

    print("\n#################\n")

    print("-4,-7,-3, 120degrees")
    goalPose = EulerPose(-4,-7,-3, 2 * (np.pi) / 3)
    print(s.getRelMovementStrafe(goalPose))

    print("\n#################\n")

    print("7,6,-3, -90 degrees")
    goalPose = EulerPose(7,6,-3, -np.pi / 2)
    print(s.getRelMovementStrafe(goalPose))

    print("\n#################\n")

    print("-7,6,-3, -90 degrees")
    goalPose = EulerPose(-7,6,-3, -np.pi / 2)
    print(s.getRelMovementStrafe(goalPose))

def testRelMoveNoStrafe2():
    print("initial pose: -1,-1,0,90degrees")
    curPose = EulerPose(-1,-1,0,np.pi / 2)
    s = SimplePlanner(curPose)
    
    print("GoalPose: 1,1,-3, -45 degrees")    
    goalPose = EulerPose(1,1,-3, -np.pi / 4)
    print('Mov: ' + str(s.getRelMovementNoStrafe(goalPose)))
#    print('CorMov: -3,45, None, pos#, -45')
    print('---Test 2---')
    print("GoalPose: -1,1,-3, -135 degrees")    
    goalPose = EulerPose(-1,1,-3, (-3 * np.pi) / 4)
    print('Mov: ' + str(s.getRelMovementNoStrafe(goalPose)))
#    print('CorMov: -3,135, None, pos#, -135')
    print('---Test 3---')
    print("GoalPose: -1,-1,-3, -135 degrees")    
    goalPose = EulerPose(-1,-1,-3, (-3 * np.pi) / 4)
    print('Mov: ' + str(s.getRelMovementNoStrafe(goalPose)))
#    print('-3,-135, None, pos#, -135')
    print('---Test 4---')
    print("GoalPose: 1,-1,-3, -45 degrees")    
    goalPose = EulerPose(1,-1,-3, (-np.pi) / 4)
    print('Mov: ' + str(s.getRelMovementNoStrafe(goalPose)))
#    print('-3,-45, None, pos#, -45')
    
def testRelMoveNoStrafe():
    print("initial pose: 0,0,0,90degrees")
    curPose = EulerPose(0,0,0,np.pi / 2)
    s = SimplePlanner(curPose)

    print("1,1,-3, 90 degrees")
    goalPose = EulerPose(1,1,-3, np.pi / 2)
    print(s.getRelMovementNoStrafe(goalPose))

    print("\n#################\n")
    
    print("1,1,-3, 180 degrees")
    goalPose = EulerPose(1,1,-3,np.pi)
    print(s.getRelMovementNoStrafe(goalPose))

    print("\n#################\n")

    # more different kinds of tests...
    print("1,1,-3, -180 degrees")
    goalPose = EulerPose(1,1,-3,-np.pi)
    print(s.getRelMovementNoStrafe(goalPose))

    print("\n#################\n")

    print("-4,-7,-3, 120degrees")
    goalPose = EulerPose(-4,-7,-3, 2 * (np.pi) / 3)
    print(s.getRelMovementNoStrafe(goalPose))

    print("\n#################\n")

    print("7,6,-3, -90 degrees")
    goalPose = EulerPose(7,6,-3, -np.pi / 2)
    print(s.getRelMovementNoStrafe(goalPose))

    print("\n#################\n")

    print("-7,6,-3, -90 degrees")
    goalPose = EulerPose(-7,6,-3, -np.pi / 2)
    print(s.getRelMovementNoStrafe(goalPose))
   
if __name__ == "__main__":
#    testRelMoveNoStrafe()
    testRelMoveNoStrafe2()    
#    testRelMoveStrafe()
