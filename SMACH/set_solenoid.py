#!/usr/bin/env python

# Patrik Vavra 2019

import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from std_srvs.srv import TriggerResponse
from geometry_msgs.msg import PoseStamped


class SetSolenoid(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[])

    
        with self:            
            smach.StateMachine.add('SET_SOLENOID',
                                ServiceState('/set_solenoid',
                                    Trigger,
                                    request_cb = SetSolenoid.request_cb,
                                    response_cb = SetSolenoid.response_cb,
                                    input_keys = []),
                                    transitions={'succeeded':'succeeded', 'aborted': 'aborted',
                                    'preempted': 'preempted'})

    @staticmethod
    @smach.cb_interface(
        input_keys=[],
        output_keys=[])
    def request_cb(userdata, request):
        solenoid_request = TriggerRequest()
        return solenoid_request

    @staticmethod
    @smach.cb_interface(
        output_keys=[],
        outcomes=['succeeded', 'aborted', 'preempted'])
    def response_cb(userdata, response):
        if response.success == True:
            return 'succeeded'
        else:
            if response.message == 'preempted':
                return 'preempted'
            return 'aborted'




