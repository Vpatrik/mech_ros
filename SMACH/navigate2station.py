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



from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult

from wait_for_goal import WaitForGoal
from wait_for_charge import WaitForCharge
from wait_for_endstop import WaitForEndstop
from geometry_msgs.msg import PoseStamped


class Navigate2Station(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self,
            outcomes=['succeeded', 'aborted'],
            input_keys=['recovery_flag'],
            output_keys = [],
            default_outcome = 'succeeded',
            child_termination_cb = Navigate2Station.child_term_cb,
            outcome_cb = Navigate2Station.out_cb)

    
        with self:

            smach.Concurrence.add('SIMPLE_NAVIGATION',
                            SimpleNavigation(),
                            remapping={})

            smach.Concurrence.add('WAIT_FOR_ENDSTOP',
                            WaitForEndstop(),
                            remapping={})


    # gets called when ANY child state terminates
    @staticmethod           
    def child_term_cb(outcome_map):

        # terminate all running states if WAIT_FOR_ENDSTOP finished with outcome 'endstop_hit'
        if outcome_map['WAIT_FOR_ENDSTOP'] == 'endstop_hit':
            return True

        if outcome_map['SIMPLE_NAVIGATION'] == 'preempted':
            return True


        # in all other case, just keep running, don't terminate anything
        return False


    # gets called when ALL child states are terminated
    @staticmethod
    def out_cb(outcome_map):
        # if outcome_map['WAIT_FOR_CHARGE'] == 'charging':
        #     return 'received_goal'
        # if outcome_map['WAIT_FOR_GOAL'] == 'received_goal':
        #     return 'received_goal'
        # else:
        #     return 'preempted'
        if outcome_map['WAIT_FOR_ENDSTOP'] == 'endstop_hit':
            return 'succeeded'
        else:
            return 'aborted'
            


class SimpleNavigation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['recovery_flag'])

        with self:
            smach.StateMachine.add('GET_PATH',
                                smach_ros.SimpleActionState('move_base_flex/get_path',
                                                            GetPathAction,
                                                            goal_cb = SimpleNavigation.get_path_goal_cb,
                                                            result_cb = SimpleNavigation.get_path_result_cb),
                                transitions={'success': 'EXE_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('EXE_PATH',
                                smach_ros.SimpleActionState('move_base_flex/exe_path',
                                                            ExePathAction,
                                                            goal_cb = SimpleNavigation.ex_path_goal_cb,
                                                            result_cb = SimpleNavigation.ex_path_result_cb),
                                transitions={'success': 'succeeded',
                                                'aborted': 'RECOVERY',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('RECOVERY',
                                smach_ros.SimpleActionState('move_base_flex/recovery',
                                                            RecoveryAction,
                                                            goal_cb = SimpleNavigation.recovery_goal_cb,
                                                            result_cb = SimpleNavigation.recovery_result_cb),
                                transitions={'success': 'GET_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

    @staticmethod
    @smach.cb_interface()
    def get_path_goal_cb(userdata, goal):
        goal.use_start_pose = False
        goal.tolerance = 0.01
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = -5.3
        goal.target_pose.pose.position.y = -4.5
        goal.target_pose.pose.orientation.z = 0.707106781
        goal.target_pose.pose.orientation.w = 0.707106781
        goal.planner = 'Charging_station_planner'

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'path'],
        outcomes=['success', 'aborted'])
    def get_path_result_cb(userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path
        if result.outcome == GetPathResult.SUCCESS:
            return 'success'
        else:
            return 'aborted'

    @staticmethod
    @smach.cb_interface(input_keys=['path'])
    def ex_path_goal_cb(userdata, goal):
        goal.path = userdata.path
        goal.controller = 'pose_follower'

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
        outcomes=['success', 'aborted'])
    def ex_path_result_cb(userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.dist_to_goal = result.dist_to_goal
        userdata.final_pose = result.final_pose
        if result.outcome == ExePathResult.SUCCESS:
            return 'success'
        elif result.outcome == ExePathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

    @staticmethod
    @smach.cb_interface(input_keys=['recovery_flag'], output_keys=['recovery_flag'])
    def recovery_goal_cb(userdata, goal):
        # TODO implement a more clever way to call the right behavior
        if not userdata.recovery_flag:
            goal.behavior = 'clear_costmap'
            userdata.recovery_flag = True
        else:
            goal.behavior = 'rotate_recovery'
            userdata.recovery_flag = False

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message'],
        outcomes=['success', 'aborted'])
    def recovery_result_cb(userdata, status, result):
        if result.outcome == RecoveryResult.SUCCESS:
            return 'success'
        else:
            return 'aborted'