#!/usr/bin/env python

# Patrik Vavra 2019

import rospy
import smach
import smach_ros

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface

from smach_ros import ServiceState
from robot_localization.srv import SetPose
from robot_localization.srv import SetPoseRequest
from robot_localization.srv import SetPoseResponse



class SetPoseKalman(smach.StateMachine):
    def __init__(self, node = None, pose = None):
        smach.StateMachine.__init__(self,
            outcomes=['pose_set', 'preempted'],
            input_keys=[])
        self.pose = pose
        self.node = node
    
        with self:            
            smach.StateMachine.add('SET_SOLENOID',
                                ServiceState('self.node',
                                    SetPose,
                                    request_cb = self.request_cb,
                                    response_cb = self.response_cb,
                                    input_keys = []),
                                    transitions={'succeeded':'pose_set', 'preempted': 'preempted'})


    @smach.cb_interface(
        input_keys=[],
        output_keys=[])
    def request_cb(self,userdata, request):
        pose_request = SetPoseRequest(self.pose)
        return pose_request


    @smach.cb_interface(
        output_keys=[],
        outcomes=['succeeded', 'aborted', 'preempted'])
    def response_cb(self,userdata, response):
        if response.success == True:
            return 'succeeded'
        else:
            if response.message == 'preempted':
                return 'preempted'
            return 'aborted'




