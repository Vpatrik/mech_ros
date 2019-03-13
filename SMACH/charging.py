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
from wait_for_charge import WaitForCharge
from wait_for_recharge import WaitForRecharge
from after_charging import AfterCharging
from navigate import Navigate
from navigate2station import Navigate2Station
from set_solenoid import SetSolenoid

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

def main():
    rospy.init_node('mbf_state_machine')

    mbf_sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    mbf_sm.userdata.sm_recovery_flag = False

    with mbf_sm:


        # gets called when ANY child state terminates
        def navig_child_term_cb(outcome_map):

            # terminate all running states if WAIT_FOR_CHARGE finished with outcome 'charging'
            if outcome_map['WAIT_FOR_CHARGE'] == 'charging':
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
            if outcome_map['WAIT_FOR_CHARGE'] == 'charging':
                return 'navigate2charge'
            if outcome_map['WHOLE_NAVIGATION'] == 'succeeded':
                output_pose = PoseStamped()
                navig_cc.userdata.sm_pose = output_pose
                output_charge_signal = Bool()
                output_charge_signal.data = False
                navig_cc.userdata.sm_charge = output_charge_signal
                return 'loop'
            if outcome_map['WHOLE_NAVIGATION'] == 'aborted':
                return 'aborted'
            else:
                return 'preempted' 
                
        # creating the concurrence state machine
        navig_cc = smach.Concurrence(outcomes = ['navigate2charge', 'preempted', 'aborted', 'loop'],
                        output_keys = ['sm_pose', 'sm_charge'],
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
                                        WaitForGoal(),
                                        transitions= {'received_goal': 'NAVIGATION', 'preempted': 'preempted'},
                                        remapping= {'target_pose': 'st_pose', 'charge': 'st_charge'})               

                smach.StateMachine.add('NAVIGATION',
                                        Navigate(),
                                        transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                        'aborted': 'aborted','charging': 'charging'},
                                        remapping={'received_goal': 'st_pose', 'charge': 'st_charge',
                                        'recovery_flag': 'sm_recovery_flag'})

            smach.Concurrence.add('WHOLE_NAVIGATION',
                                whole_navig,
                                remapping = {})

            smach.Concurrence.add('WAIT_FOR_CHARGE',
                               WaitForCharge(),
                               remapping={'target_pose': 'sm_pose','charge': 'sm_charge'})

        smach.StateMachine.add('NAVIGATION_LOOP',
                               navig_cc,
                               transitions={'navigate2charge': 'NAVIGATE_BEFORE_STATION', 'loop': 'NAVIGATION_LOOP',
                                'preempted': 'preempted','aborted': 'aborted'},
                                remapping = {'sm_pose': 'sm_pose', 'sm_charge': 'sm_charge',
                                'sm_recovery_flag': 'sm_recovery_flag'})

        smach.StateMachine.add('NAVIGATE_BEFORE_STATION',
                               Navigate(),
                               transitions={'succeeded': 'NAVIGATION_LOOP', 'preempted': 'preempted',
                               'aborted': 'aborted','charging': 'NAVIGATE_2_PLUG'},
                               remapping={'received_goal': 'sm_pose', 'charge': 'sm_charge',
                               'recovery_flag': 'sm_recovery_flag'})

        smach.StateMachine.add('NAVIGATE_2_PLUG',
                               Navigate2Station(),
                               transitions={'succeeded': 'CHARGING',
                               'aborted': 'aborted'},
                               remapping={'recovery_flag': 'sm_recovery_flag', 'charge': 'sm_charge'})


        smach.StateMachine.add('CHARGING',
                               AfterCharging(),
                               transitions={'succeeded': 'NAVIGATION_LOOP',
                                'preempted': 'preempted','aborted': 'aborted'},
                                remapping = {})


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