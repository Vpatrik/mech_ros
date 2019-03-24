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
# Main structure from above copyright source code was taken and modified for custom needs

"""
State machine that navigates to goal from userdata or from initialization, with given specifications
"""

import rospy
import smach
import smach_ros
import dynamic_reconfigure.client

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult

from wait_for_goal import WaitForGoal
from geometry_msgs.msg import PoseStamped


class Navigate(smach.StateMachine):
    def __init__(self, charge = False, goal_pose = None, planner = 'Normal_planner', controller = 'dwa',
        reconfigure_smaller = None, reconfigure_bigger = None):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'preempted','aborted'],
            input_keys=['received_goal','recovery_flag'] if goal_pose is None else ['recovery_flag'])

        self.charge = charge
        self.goal_pose = goal_pose
        self.planner = planner
        self.controller = controller
        self.reconfigure_smaller = reconfigure_smaller
        self.reconfigure_bigger = reconfigure_bigger
    
        with self:            
            smach.StateMachine.add('GET_PATH',
                                smach_ros.SimpleActionState('move_base_flex/get_path',
                                                            GetPathAction,
                                                            goal_cb = self.get_path_goal_cb,
                                                            result_cb = self.get_path_result_cb),
                                transitions={'success': 'EXE_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('EXE_PATH',
                                smach_ros.SimpleActionState('move_base_flex/exe_path',
                                                            ExePathAction,
                                                            goal_cb = self.ex_path_goal_cb,
                                                            result_cb = self.ex_path_result_cb),
                                transitions={'success': 'succeeded',
                                                'aborted': 'RECOVERY',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('RECOVERY',
                                smach_ros.SimpleActionState('move_base_flex/recovery',
                                                            RecoveryAction,
                                                            goal_cb = self.recovery_goal_cb,
                                                            result_cb = self.recovery_result_cb),
                                transitions={'success': 'GET_PATH',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

    @cb_interface(
        input_keys=['received_goal'])
    def get_path_goal_cb(self, userdata, goal):

        if self.reconfigure_smaller:
            # Dynamicaly reconfigure robot footprint to fit in the charging station
            client_local_costmap = dynamic_reconfigure.client.Client('/move_base_flex/local_costmap')
            client_global_costmap = dynamic_reconfigure.client.Client('/move_base_flex/global_costmap')
            client_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base_flex/local_costmap/inflation')
            client_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base_flex/global_costmap/inflation')

            # Original paramateres
            # /move_base_flex/local_costmap/inflation/footprint [[-0.12,-0.1],[-0.12,0.1],[0.12,0.1],[0.12,-0.1]]
            # /move_base_flex/local_costmap/inflation/robot_radius 0.46 - probbly not used
            # /move_base_flex/local_costmap/inflation/cost_scaling_factor 2.55
            # /move_base_flex/local_costmap/inflation/inflation_radius 0.38
            # /move_base_flex/local_costmap/inflation/enabled True
            # /move_base_flex/global_costmap/inflation/footprint [[-0.12,-0.1],[-0.12,0.1],[0.12,0.1],[0.12,-0.1]]
            # /move_base_flex/global_costmap/inflation/robot_radius 0.46 - probbly not used
            # /move_base_flex/global_costmap/inflation/cost_scaling_factor 2.55
            # /move_base_flex/global_costmap/inflation/inflation_radius 0.38
            # /move_base_flex/global_costmap/inflation/enabled True

            # Probably would be best to disable inflation layer in both costmaps for navigation to plug
            # and to downscale footprint
            params_local_footprint = {'footprint' : [[-0.06,-0.05],[-0.06,0.05],[0.06,0.05],[0.06,-0.05]]}
            params_global_footprint = {'footprint' : [[-0.06,-0.05],[-0.06,0.05],[0.06,0.05],[0.06,-0.05]]}
            params_local_inflation = {'enabled' : False}
            params_global_inflation = {'enabled' : False}


            config_local_costmap = client_local_costmap.update_configuration(params_local_footprint)
            config_global_costmap = client_global_costmap.update_configuration(params_global_footprint)
            config_local_inflation = client_local_costmap_inflation.update_configuration(params_local_inflation)
            config_global_inflation = client_global_costmap_inflation.update_configuration(params_global_inflation)


        if self.goal_pose is None:
            target_pose = userdata.received_goal
            goal.tolerance = 0.2

        else:           
            target_pose = self.goal_pose
            if self.charge == False:
                goal.tolerance = 0.08
            else:
                goal.tolerance = 0.2
        

        target_pose.header.stamp = rospy.Time.now()
        goal.target_pose = target_pose
        goal.use_start_pose = False
        goal.planner = self.planner
        rospy.loginfo("Navigating to position: (%s, %s, %s)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
        goal.target_pose.pose.position.z)
        rospy.loginfo("Navigating to final orientation: (%s, %s, %s, %s)", goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w)

    @cb_interface(
        output_keys=['outcome', 'message', 'path'],
        outcomes=['success', 'aborted', 'preempted'])
    def get_path_result_cb(self, userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path
        if result.outcome == GetPathResult.SUCCESS:
            return 'success'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

    @cb_interface(input_keys=['path'])
    def ex_path_goal_cb(self, userdata, goal):
        goal.path = userdata.path
        goal.controller = self.controller

    @cb_interface(input_keys=[],
        output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
        outcomes=['success', 'aborted', 'preempted'])
    def ex_path_result_cb(self, userdata, status, result):
        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.dist_to_goal = result.dist_to_goal
        userdata.final_pose = result.final_pose
        if result.outcome == ExePathResult.SUCCESS:

            if self.reconfigure_bigger:
                # Dynamicaly reconfigure navigation parameters to normal state
                client_local_costmap = dynamic_reconfigure.client.Client('/move_base_flex/local_costmap')
                client_global_costmap = dynamic_reconfigure.client.Client('/move_base_flex/global_costmap')
                client_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base_flex/local_costmap/inflation')
                client_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base_flex/global_costmap/inflation')

                params_local_footprint = {'footprint' : [[-0.12,-0.1],[-0.12,0.1],[0.12,0.1],[0.12,-0.1]]}
                params_global_footprint = {'footprint' : [[-0.12,-0.1],[-0.12,0.1],[0.12,0.1],[0.12,-0.1]]}
                params_local_inflation = {'enabled' : True}
                params_global_inflation = {'enabled' : True}


                config_local_costmap = client_local_costmap.update_configuration(params_local_footprint)
                config_global_costmap = client_global_costmap.update_configuration(params_global_footprint)
                config_local_inflation = client_local_costmap_inflation.update_configuration(params_local_inflation)
                config_global_inflation = client_global_costmap_inflation.update_configuration(params_global_inflation)

            return 'success'
        elif result.outcome == ExePathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'

    @cb_interface(input_keys=['recovery_flag'], output_keys=['recovery_flag'])
    def recovery_goal_cb(self, userdata, goal):

        if not userdata.recovery_flag:
            goal.behavior = 'moveback_recovery'
            userdata.recovery_flag = True
        else:
            goal.behavior = 'rotate_recovery'
            userdata.recovery_flag = False

    @cb_interface(
        output_keys=['outcome', 'message'],
        outcomes=['success', 'aborted', 'preempted'])
    def recovery_result_cb(self, userdata, status, result):
        if result.outcome == RecoveryResult.SUCCESS:
            return 'success'
        elif result.outcome == RecoveryResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'
