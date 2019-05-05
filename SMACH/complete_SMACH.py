#!/usr/bin/env python

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
from monitor_battery import WaitForBatteryLevel
from wait_for_endstop import WaitForEndstop
from wait_for_timed_out import TimedOut
from charging import Charging
from navigate import Navigate
from send_velocity_command import PublishVelocity

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('mbf_state_machine')
    before_station = PoseStamped()
    plug = PoseWithCovarianceStamped()
    plug_navig = PoseStamped()


    # ROS adjustable parameters
    timed_out = rospy.get_param("~timed_out", 100)
    lower_battery_threshold = rospy.get_param("~lower_battery_threshold", 3.5)
    higher_battery_threshold = rospy.get_param("~higher_battery_threshold", 4.5)
    before_station_list = rospy.get_param("~pose_before_station", [-5.3,-3.5,0,0,0,0.707106781,0.707106781])
    plug_navig_list = rospy.get_param("~pose_plug_navig", [-5.3,-4.5,0,0,0,0.707106781,0.707106781])
    plug_list = rospy.get_param("~pose_plug", [-5.3,-4.5,0,0,0,0.707106781,0.707106781])
    plug_covariance= rospy.get_param("~pose_plug_covariance",   [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01])
    world_frame = rospy.get_param("~world_frame", 'map')
    simulation = rospy.get_param("~simulation", False)
    repetitions = rospy.get_param("~repetitions", 2)

    # # Convert pose from list data type to ROS PoseStamped
    before_station.header.frame_id = world_frame
    before_station.pose.position.x,before_station.pose.position.y,before_station.pose.position.z = before_station_list[0:3]
    before_station.pose.orientation.x,before_station.pose.orientation.y,before_station.pose.orientation.z,before_station.pose.orientation.w = before_station_list[3:7]
    
    plug_navig.header.frame_id = world_frame
    plug_navig.pose.position.x,plug_navig.pose.position.y,plug_navig.pose.position.z = plug_navig_list[0:3]
    plug_navig.pose.orientation.x,plug_navig.pose.orientation.y,plug_navig.pose.orientation.z,plug_navig.pose.orientation.w = plug_navig_list[3:7]

    plug.header.frame_id = world_frame
    plug.pose.pose.position.x,plug.pose.pose.position.y,plug.pose.pose.position.z = plug_list[0:3]
    plug.pose.pose.orientation.x,plug.pose.pose.orientation.y,plug.pose.pose.orientation.z,plug.pose.pose.orientation.w = plug_list[3:7]
    # Fill covariance data for plug for setting pose to Kalman filters
    plug.pose.covariance = plug_covariance

    # initialize SMACH wth userdara
    mbf_sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    mbf_sm.userdata.sm_recovery_flag = False
    mbf_sm.userdata.sm_number = 0

    with mbf_sm:

        # gets called when ANY child state terminates
        def navig_child_term_cb(outcome_map):

            # terminate all running states if WAIT_FOR_CHARGE finished with outcome 'charging'
            if outcome_map['WAIT_FOR_CHARGE'] == 'level_reached':
                return True

            # terminate all running states if NAVIGATION finished with outcome 'succeeded'
            if outcome_map['WHOLE_NAVIGATION'] == 'succeeded':
                return True

            # terminate all running states if NAVIGATION finished with outcome 'aborted'
            if outcome_map['WHOLE_NAVIGATION'] == 'aborted':
                return True

            # in all other case, just keep running, don't terminate anything
            return False


        # gets called when ALL child states are terminated
        def navig_out_cb(outcome_map):
            if outcome_map['WAIT_FOR_CHARGE'] == 'level_reached':
                return 'charge'
            if outcome_map['WHOLE_NAVIGATION'] == 'succeeded':
                return 'loop'
            if outcome_map['WHOLE_NAVIGATION'] == 'aborted':
                return 'aborted'
            else:
                return 'preempted'

        # creating the concurrence state machine
        navig_cc = smach.Concurrence(outcomes = ['charge', 'preempted', 'aborted', 'loop'],
                        output_keys = [],
                        input_keys=['sm_recovery_flag'],
                        default_outcome = 'preempted',
                        child_termination_cb = navig_child_term_cb,
                        outcome_cb = navig_out_cb)

        with navig_cc:

            whole_navig = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'charging'],
            input_keys=['sm_recovery_flag'],
            output_keys= [])

            with whole_navig:
                smach.StateMachine.add('WAIT_FOR_GOAL',
                                        WaitForGoal(simulation= simulation),
                                        transitions= {'received_goal': 'NAVIGATION', 'preempted': 'preempted'},
                                        remapping= {'target_pose': 'st_pose'})               

                smach.StateMachine.add('NAVIGATION',
                                        Navigate(charge= False, goal_pose= None, planner= 'Normal_planner',
                                        controller= 'dwa'),
                                        transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                        'aborted': 'aborted'},
                                        remapping={'received_goal': 'st_pose',
                                        'recovery_flag': 'sm_recovery_flag'})

            smach.Concurrence.add('WHOLE_NAVIGATION',
                                whole_navig,
                                remapping = {})

            smach.Concurrence.add('WAIT_FOR_CHARGE',
                               WaitForBatteryLevel(lower = True, threshold = lower_battery_threshold),
                               remapping={})
                               
        smach.StateMachine.add('NAVIGATION_LOOP',
                               navig_cc,
                               transitions={'charge': 'NAVIGATION_TO_CHARGE_WITH_CONTROL', 'loop': 'NAVIGATION_LOOP',
                                'preempted': 'preempted','aborted': 'aborted'},
                                remapping = {'sm_recovery_flag': 'sm_recovery_flag'})

            ############ Complete navigation from arbitrary place to plug in charging station 
            # - without control of timing out ###############################################

        # gets called when ANY child state terminates
        def navig_charge_child_term_cb(outcome_map):

            # terminate all running states if TIMING_OUT finished with outcome 'timed_out'
            if outcome_map['TIMING_OUT'] == 'timed_out':
                return True

            if outcome_map['TIMING_OUT'] == 'aborted':
                return True

            # terminate all running states if NAVIGATION finished with outcome 'succeeded'
            if outcome_map['COMPLETE_NAVIGATION_FOR_CHARGING'] in ('succeeded','aborted', 'preempted'):
                return True
            
            # in all other case, just keep running, don't terminate anything
            return False


        # gets called when ALL child states are terminated
        def navig_charge_out_cb(outcome_map):
            if outcome_map['COMPLETE_NAVIGATION_FOR_CHARGING'] == 'succeeded':
                return 'succeeded'

            if outcome_map['TIMING_OUT'] == 'timed_out':
                return 'retry'

            if outcome_map['TIMING_OUT'] == 'aborted':
                return 'aborted'

            for state in ('COMPLETE_NAVIGATION_FOR_CHARGING', 'TIMING_OUT'):
                if outcome_map[state] == 'preempted':
                    return 'preempted'

            else:
                return 'aborted'
                
        navig_charge = smach.Concurrence(outcomes = ['succeeded', 'preempted', 'aborted', 'retry'],
                        output_keys = ['sm_number_out'],
                        input_keys=['sm_recovery_flag', 'sm_number_in'],
                        default_outcome = 'preempted',
                        child_termination_cb = navig_charge_child_term_cb,
                        outcome_cb = navig_charge_out_cb)

        with navig_charge:

                ############# Navigation from arbitrary place to place before charging station #################

            navigation = smach.StateMachine(outcomes=['preempted', 'aborted', 'succeeded'],
            input_keys=['sm_recovery_flag'],
            output_keys= [])

            with navigation:

                # Fill time informmation in pose 
                before_station.header.stamp = rospy.Time.now()
                
                smach.StateMachine.add('NAVIGATE_BEFORE_STATION',
                                    Navigate(charge= True, goal_pose= before_station, planner= 'Normal_planner',
                                    controller= 'dwa'),
                                    transitions={'succeeded': 'NAVIGATE_TO_PLUG_AND_CONTROL', 'preempted': 'preempted',
                                    'aborted': 'aborted'},
                                    remapping={'recovery_flag': 'sm_recovery_flag'})

                ############# End of navigation from arbitrary place to place before charging station #################

                #################### Navigation to plug  - can be successfull only with endstop is hit ###############
                
                # gets called when ANY child state terminates        
                def plug_child_term_cb(outcome_map):

                    # terminate all running states if WAIT_FOR_ENDSTOP finished with outcome 'endstop_hit'
                    if outcome_map['WAIT_FOR_ENDSTOP'] == 'endstop_hit':
                        return True

                    if outcome_map['NAVIGATE_2_PLUG'] == 'preempted':
                        return True

                    # in all other case, just keep running, don't terminate anything
                    return False


                # gets called when ALL child states are terminated
                def plug_out_cb(outcome_map):
                    if outcome_map['WAIT_FOR_ENDSTOP'] == 'endstop_hit':
                        return 'succeeded'
                    if outcome_map['NAVIGATE_2_PLUG'] == 'preempted':
                        return 'preempted'
                    else:
                        return 'aborted'

                navig_to_plug = smach.Concurrence(
                    outcomes=['succeeded', 'aborted', 'preempted'],
                    input_keys=['sm_recovery_flag'],
                    output_keys = [],
                    default_outcome = 'succeeded',
                    child_termination_cb = plug_child_term_cb,
                    outcome_cb = plug_out_cb)

                with navig_to_plug:

                    # Fill time informmation in pose
                    plug.header.stamp = rospy.Time.now()

                    smach.Concurrence.add('NAVIGATE_2_PLUG',
                                        Navigate(charge= True, goal_pose= plug_navig, planner= 'Charging_station_planner',
                                        controller= 'dwa_station', reconfigure_smaller= True),
                                        remapping={'recovery_flag': 'sm_recovery_flag'})
                   
                    smach.Concurrence.add('WAIT_FOR_ENDSTOP',
                                    WaitForEndstop(),
                                    remapping={})

                ################ END of navigation to plug #######################

                smach.StateMachine.add('NAVIGATE_TO_PLUG_AND_CONTROL',
                                    navig_to_plug,
                                    transitions={'succeeded': 'STOP_ROBOT', 'preempted': 'preempted',
                                    'aborted': 'aborted'},
                                    remapping={'recovery_flag': 'sm_recovery_flag'})

                # Initialize velocity command for stopping robot
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0

                smach.StateMachine.add('STOP_ROBOT',
                                    PublishVelocity(cmd_vel),
                                    transitions={'velocity_published': 'succeeded',
                                    'preempted': 'preempted'},
                                    remapping={})

            smach.Concurrence.add('COMPLETE_NAVIGATION_FOR_CHARGING',
                                    navigation,
                                    remapping={'sm_recovery_flag': 'sm_recovery_flag'})

            ################ END of complete navigation from arbitrary place to plug in charging station #######################

            smach.Concurrence.add('TIMING_OUT',
                                TimedOut(time = timed_out, repetitions= repetitions),
                                remapping={'number_in': 'sm_number_in', 'number_out': 'sm_number_out'})

        smach.StateMachine.add('NAVIGATION_TO_CHARGE_WITH_CONTROL',
                                navig_charge,
                                transitions={'succeeded': 'CHARGING', 'retry': 'NAVIGATION_TO_CHARGE_WITH_CONTROL',
                               'preempted': 'preempted', 'aborted':'aborted'},
                               remapping={'sm_recovery_flag':'sm_recovery_flag', 'sm_number_in': 'sm_number',
                               'sm_number_out': 'sm_number'})                           


            ################ END of complete navigation from arbitrary place to plug in charging station
            # included timing out #######################              

        smach.StateMachine.add('CHARGING',
                               Charging(charge_level= higher_battery_threshold, before_station = before_station, pose= plug),
                               transitions={'succeeded': 'NAVIGATION_LOOP',
                                'preempted': 'preempted','aborted': 'aborted'},
                                remapping = {'sm_recovery_flag' : 'sm_recovery_flag'})


    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('smach_server', mbf_sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = mbf_sm.execute()

    # Wait for interrupt and stop introspection server
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()