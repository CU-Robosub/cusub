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

    def __init__(self, task_name):
        self.name = task_name
        super(VisitTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(self.getPrior(), rospy.get_param("tasks/" + self.name + "/search_alg"), "cusub_cortex/mapper_out/"+task_name)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg, topic):
        self.search = Search(searchAlg, prior, topic)

    def linkObjectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'task_aborted', 'success':'task_success'})
