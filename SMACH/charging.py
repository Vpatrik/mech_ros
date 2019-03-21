#!/usr/bin/env python


## Copyright (c) 2018, Sebastian Putz
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Patrik Vavra 2019

import rospy
import smach
import smach_ros

from navigate2station import SimpleNavigation
from wait_for_recharge import WaitForRecharge
from send_velocity_command import PublishVelocity

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from smach_ros import ServiceState
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from std_srvs.srv import TriggerResponse

class Charging(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[])

        # Initialize velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.05
        cmd_vel.angular.z = 0

        with self:
            smach.StateMachine.add('WAIT_FOR_RECHARGE',
                                WaitForRecharge(),
                                transitions={'charged': 'MOVE_BACKWARDS',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = { 'pose': 'sm_pose', 'charge': 'sm_charge',
                                    'recovery': 'sm_recovery_flag'})

            smach.StateMachine.add('MOVE_BACKWARDS',
                                PublishVelocity(cmd_vel),
                                transitions={'velocity_published': 'UNLOCK_SOLENOID',
                                    'preempted': 'preempted'},
                                    remapping = {})                                    

            smach.StateMachine.add('UNLOCK_SOLENOID',
                                ServiceState('/set_solenoid',
                                    Trigger,
                                    request_cb = Charging.request_cb,
                                    response_cb = Charging.response_cb,
                                    input_keys = []),
                                transitions={'succeeded': 'NAVIGATE',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = {})

            smach.StateMachine.add('NAVIGATE',
                                SimpleNavigation(),
                                transitions={'succeeded': 'succeeded',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = {'sm_pose': 'received_goal', 'charge': 'sm_charge',
                                    'sm_recovery_flag': 'recovery_flag'})

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


