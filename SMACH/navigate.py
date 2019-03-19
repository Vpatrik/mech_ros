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
from geometry_msgs.msg import PoseStamped


class Navigate(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'preempted', 'aborted', 'charging'],
            input_keys=['received_goal','charge', 'recovery_flag'])

    
        with self:            
            smach.StateMachine.add('GET_PATH',
                                smach_ros.SimpleActionState('move_base_flex/get_path',
                                                            GetPathAction,
                                                            goal_cb = Navigate.get_path_goal_cb,
                                                            result_cb = Navigate.get_path_result_cb),
                                transitions={'success': 'EXE_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('EXE_PATH',
                                smach_ros.SimpleActionState('move_base_flex/exe_path',
                                                            ExePathAction,
                                                            goal_cb = Navigate.ex_path_goal_cb,
                                                            result_cb = Navigate.ex_path_result_cb),
                                transitions={'success': 'succeeded',
                                                'aborted': 'RECOVERY',
                                                'charging': 'charging',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('RECOVERY',
                                smach_ros.SimpleActionState('move_base_flex/recovery',
                                                            RecoveryAction,
                                                            goal_cb = Navigate.recovery_goal_cb,
                                                            result_cb = Navigate.recovery_result_cb),
                                transitions={'success': 'GET_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

    @staticmethod
    @smach.cb_interface(
        input_keys=['received_goal', 'charge'])
    def get_path_goal_cb(userdata, goal):
        goal.use_start_pose = False

        if userdata.charge.data == True:
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = 'map'

            # Simulation
            # target_pose.pose.position.x = -5.3
            # target_pose.pose.position.y = -3.5
            # target_pose.pose.orientation.z = 0.707106781
            # target_pose.pose.orientation.w = 0.707106781

            # Real robot
            target_pose.pose.position.x = 2.2
            target_pose.pose.position.y = 0.085
            target_pose.pose.orientation.z = 0.707106781
            target_pose.pose.orientation.w = 0.707106781

            goal.target_pose = target_pose
            goal.planner = 'Normal_planner'
            goal.tolerance = 0.05

        else: 
            goal.target_pose = userdata.received_goal
            goal.tolerance = 0.2
        goal.planner = 'Normal_planner'

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'path'],
        outcomes=['success', 'aborted', 'preempted'])
    def get_path_result_cb(userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path
        if result.outcome == GetPathResult.SUCCESS:
            return 'success'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

    @staticmethod
    @smach.cb_interface(input_keys=['path'])
    def ex_path_goal_cb(userdata, goal):
        goal.path = userdata.path
        goal.controller = 'dwa'

    @staticmethod
    @smach.cb_interface(input_keys=['charge'],
        output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
        outcomes=['success', 'aborted', 'charging', 'preempted'])
    def ex_path_result_cb(userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.dist_to_goal = result.dist_to_goal
        userdata.final_pose = result.final_pose
        if result.outcome == ExePathResult.SUCCESS:
            if userdata.charge.data == True:
                return 'charging'
            return 'success'
        elif result.outcome == ExePathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

    @staticmethod
    @smach.cb_interface(input_keys=['recovery_flag'], output_keys=['recovery_flag'])
    def recovery_goal_cb(userdata, goal):

        if not userdata.recovery_flag:
            goal.behavior = 'clear_costmap'
            userdata.recovery_flag = True
        else:
            goal.behavior = 'rotate_recovery'
            userdata.recovery_flag = False

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message'],
        outcomes=['success', 'aborted', 'preempted'])
    def recovery_result_cb(userdata, status, result):
        if result.outcome == RecoveryResult.SUCCESS:
            return 'success'
        elif result.outcome == RecoveryResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'
