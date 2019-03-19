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

# For inspiration

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


@smach.cb_interface(input_keys=['target_pose'])
def get_path_goal_cb(userdata, goal):
    goal.use_start_pose = False
    goal.tolerance = 0.2
    goal.target_pose = userdata.target_pose
    goal.planner = 'planner'


@smach.cb_interface(
    output_keys=['outcome', 'message', 'path'],
    outcomes=['success', 'failure'])
def get_path_result_cb(userdata, status, result):
    userdata.message = result.message
    userdata.outcome = result.outcome
    userdata.path = result.path
    if result.outcome == GetPathResult.SUCCESS:
        return 'success'
    else:
        return 'failure'


@smach.cb_interface(input_keys=['path'])
def ex_path_goal_cb(userdata, goal):
    goal.path = userdata.path
    goal.controller = 'controller'


@smach.cb_interface(
    output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
    outcomes=['success', 'failure'])
def ex_path_result_cb(userdata, status, result):
    userdata.message = result.message
    userdata.outcome = result.outcome
    userdata.dist_to_goal = result.dist_to_goal
    userdata.final_pose = result.final_pose
    if result.outcome == ExePathResult.SUCCESS:
        return 'success'
    else:
        return 'failure'


@smach.cb_interface(input_keys=['recovery_flag'], output_keys=['recovery_flag'])
def recovery_goal_cb(userdata, goal):
    # TODO implement a more clever way to call the right behavior
    if not userdata.recovery_flag:
        goal.behavior = 'clear_costmap'
        userdata.recovery_flag = True
    else:
        goal.behavior = 'rotate_recovery'
        userdata.recovery_flag = False


@smach.cb_interface(
    output_keys=['outcome', 'message'],
    outcomes=['success', 'failure'])
def recovery_result_cb(userdata, status, result):
    if result.outcome == RecoveryResult.SUCCESS:
        return 'success'
    else:
        return 'failure'


def main():
    rospy.init_node('mbf_state_machine')

    mbf_sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    mbf_sm.userdata.recovery_flag = False

    with mbf_sm:
        smach.StateMachine.add('WAIT_FOR_GOAL',
                               WaitForGoal(),
                               transitions={'succeeded': 'GET_PATH', 'preempted': 'preempted'})

        smach.StateMachine.add('GET_PATH',
                               smach_ros.SimpleActionState('move_base_flex/get_path',
                                                           GetPathAction,
                                                           goal_cb=get_path_goal_cb,
                                                           result_cb=get_path_result_cb),
                               transitions={'success': 'EXE_PATH',
                                            'failure': 'WAIT_FOR_GOAL'})

        smach.StateMachine.add('EXE_PATH',
                               smach_ros.SimpleActionState('move_base_flex/exe_path',
                                                           ExePathAction,
                                                           goal_cb=ex_path_goal_cb,
                                                           result_cb=ex_path_result_cb),
                               transitions={'success': 'WAIT_FOR_GOAL',
                                            'failure': 'RECOVERY'})

        smach.StateMachine.add('RECOVERY',
                               smach_ros.SimpleActionState('move_base_flex/recovery',
                                                           RecoveryAction,
                                                           goal_cb=recovery_goal_cb,
                                                           result_cb=recovery_result_cb),
                               transitions={'success': 'GET_PATH',
                                            'failure': 'WAIT_FOR_GOAL'})

    sis = smach_ros.IntrospectionServer('mbf_state_machine_server', mbf_sm, '/SM_ROOT')
    sis.start()
    outcome = mbf_sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
