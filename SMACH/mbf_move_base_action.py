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

import roslib
import rospy
import smach
import smach_ros
from mbf_msgs.msg import MoveBaseAction
from mbf_msgs.msg import MoveBaseResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from wait_for_goal import WaitForGoal


@smach.cb_interface(
    input_keys=['target_pose', 'controller', 'planner', 'recovery_behaviors'])
def move_base_goal_cb(userdata, goal):
    goal.target_pose = userdata.target_pose
    goal.controller = userdata.controller
    goal.planner = userdata.planner
    goal.recovery_behaviors = userdata.recovery_behaviors


@smach.cb_interface(
    outcomes=['success', 'general_failure', 'plan_failure', 'ctrl_failure', 'undefined_failure'],
    output_keys=['mb_outcome', 'mb_message', 'mb_dist_to_goal', 'mb_angle_to_goal'])
def move_base_result_cb(userdata, status, result):
    userdata.mb_outcome = result.outcome
    userdata.mb_message = result.message
    userdata.mb_dist_to_goal = result.dist_to_goal
    userdata.mb_angle_to_goal = result.angle_to_goal

    if result.outcome is MoveBaseResult.SUCCESS:
        return 'success'
    elif 10 <= result.outcome < 50:
        return 'general_failure'
    elif 50 <= result.outcome < 100:
        return 'plan_failure'
    elif 100 <= result.outcome < 150:
        return 'ctrl_failure'
    else:
        return 'undefined_failure'


def main():
    rospy.init_node('mbf_state_machine')

    mbf_sm = smach.StateMachine(outcomes=['aborted', 'succeeded', 'preempted'])
    mbf_sm.userdata.path = Path()
    mbf_sm.userdata.previous_state = None
    mbf_sm.userdata.act_pos = None
    mbf_sm.userdata.error = None
    mbf_sm.userdata.error_status = None
    mbf_sm.userdata.goal_position = None
    mbf_sm.userdata.recovery_behavior = None
    mbf_sm.userdata.clear_costmap_flag = False
    mbf_sm.userdata.controller = 'dwa'
    mbf_sm.userdata.planner = 'GlobalPlanner'
    mbf_sm.userdata.recovery_behaviors = ['clear_costmap', 'rotate_recovery']

    with mbf_sm:
        smach.StateMachine.add('WAIT_FOR_GOAL',
                               WaitForGoal(),
                               transitions={'received_goal': 'MOVE_BASE',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('MOVE_BASE',
                               smach_ros.SimpleActionState(
                                   '/move_base_flex/move_base',
                                   MoveBaseAction,
                                   goal_cb=move_base_goal_cb,
                                   result_cb=move_base_result_cb
                               ),
                               transitions={
                                   'success': 'WAIT_FOR_GOAL',
                                   'general_failure': 'WAIT_FOR_GOAL',
                                   'plan_failure': 'WAIT_FOR_GOAL',
                                   'ctrl_failure': 'WAIT_FOR_GOAL',
                                   'undefined_failure': 'aborted'}
                               )

    sis = smach_ros.IntrospectionServer('mbf_move_base_server', mbf_sm, '/SM_ROOT')
    sis.start()
    outcome = mbf_sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()