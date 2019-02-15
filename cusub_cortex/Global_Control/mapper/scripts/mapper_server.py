#! /usr/bin/env python

"""
Mapper node

Should this be replaced by the waypoint navigator?

"""

import rospy
import actionlib

import mapper.msg

class Mapper(object):
    # messages for publishing feedback and result
    _feedback = mapper.msg.mapperFeedback()
    _result = mapper.msg.mapperResult()

    def __init__(self):
        # create the server
        self.server = actionlib.SimpleActionServer('/mapper', mapper.msg.mapperAction,
                                                    execute_cb=self.execute, auto_start=False)

        self.server.start()

    # this is called everytime the client(smach) sends another goal
    def execute(self, goal):
        # know if not preempted
        success = True

        # give feedback using _feedback msg (not used rn)

        # start executing actions
        # need some sort of while loop, maybe listening to reacehed_goal ?

        # check if preempted
        if self.server.is_preempt_requested():
            rospy.loginfo('preempted !!!!!')
            success = False


        if success:
            # publish to result
            self._result.finish = True
            self.server.set_succeeded(self._result)







if __name__=="__main__":
    rospy.init_node('mapper')
    server =  Mapper()
    rospy.spin()
