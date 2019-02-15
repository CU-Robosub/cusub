#!/usr/bin/env python

import rospy
import smach
import smach_ros
from waypoint_navigator.msg import waypointAction, waypointGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

# search state
class Test(object):
    def __init__(self):
        print("test inited")
        self.a_sub = rospy.Subscriber('/FUCK', Empty, self.fuck_callback)
        self.fuck = False
        self.mult = 0

    def fuck_callback(self, msg):
        self.fuck = True


    def give_goal(self, userdata, default_goal):
        print("giving goal")
        goal = waypointGoal()
        pose = PoseStamped()
        pose.pose.position.x = 2 * (self.mult % 2)
        self.mult += 1
        goal.goal_pose = pose

        return goal

    def result_received(self, userdata, status, result):
        print(result)

        if self.fuck:
            return 'preempted'
        else:
            return 'succeeded'



class Test2(object):
    def __init__(self):
        print("test2 inited")

    def give_goal(self, userdata, default_goal):
        print("giving test2 goal")

        goal = waypointGoal()
        pose = PoseStamped()
        pose.pose.position.x = -6

        goal.goal_pose = pose

        return goal

    def result_received(self, userdata, status, result):
        # print(status)
        print(result)
        return 'succeeded'


def shutdown():
    print("shutting it down")

# main
def main():
    rospy.init_node('smach_main')
    rospy.on_shutdown(shutdown)

    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    t = Test()

    tt = Test2()

    with sm:
        smach.StateMachine.add('TEST_GOAL',
                                smach_ros.SimpleActionState('waypoint', waypointAction,
                                goal_cb=t.give_goal, result_cb=t.result_received),
                                {'succeeded':'TEST_GOAL', 'preempted':'TEST2_GOAL'})


        smach.StateMachine.add('TEST2_GOAL',
                                smach_ros.SimpleActionState('waypoint', waypointAction,
                                goal_cb=tt.give_goal, result_cb=tt.result_received),
                                {'succeeded':'succeeded'})



    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()


    outcome = sm.execute()

    rospy.spin()

    # sis.stop()




    rospy.signal_shutdown('All done.')


if __name__=='__main__':
    main()
