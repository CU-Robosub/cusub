#!/usr/bin/env python

"""
Impersonates a given task by going to the prior
Upon reaching the prior it does nothing and completes
"""

from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach

class VisitTask(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior, searchAlg):

        super(VisitTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(prior, searchAlg)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        self.search = Search(searchAlg, prior)

    def initMapperSubs(self):
        """
        Just going to the prior so no need to act on any pose estimates
        """
        pass

    def linkObjectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'task_aborted', 'success':'task_success'})
        
