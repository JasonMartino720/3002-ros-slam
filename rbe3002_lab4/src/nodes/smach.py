#!/usr/bin/env python2

import rospy
import smach
import smach_ros

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete'])

    def execute(self):