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
from set_solenoid import SetSolenoid
from wait_for_recharge import WaitForRecharge
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class AfterCharging(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[])


        with self:
            smach.StateMachine.add('WAIT_FOR_RECHARGE',
                                WaitForRecharge(),
                                transitions={'charged': 'UNLOCK_SOLENOID',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = { 'pose': 'sm_pose', 'charge': 'sm_charge',
                                    'recovery': 'sm_recovery_flag'})

            smach.StateMachine.add('UNLOCK_SOLENOID',
                                SetSolenoid(),
                                transitions={'succeeded': 'NAVIGATE',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = {})

            smach.StateMachine.add('NAVIGATE',
                                SimpleNavigation(),
                                transitions={'succeeded': 'succeeded',
                                    'preempted': 'preempted','aborted': 'aborted'},
                                    remapping = {'sm_pose': 'received_goal', 'charge': 'sm_charge',
                                    'sm_recovery_flag': 'recovery_flag'})



