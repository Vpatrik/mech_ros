#!/usr/bin/env python


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
from navigate import Navigate
from navigate2station import Navigate2Station

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('mbf_state_machine')

    mbf_sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    mbf_sm.userdata.recovery_flag = False

    with mbf_sm:

         # gets called when ANY child state terminates
        def child_term_cb(outcome_map):

            # terminate all running states if WAIT_FOR_CHARGE finished with outcome 'charging'
            if outcome_map['WAIT_FOR_CHARGE'] == 'charging':
                return True

            # terminate all running states if WAIT_FOR_GOAL finished with outcome 'received_goal'
            if outcome_map['WAIT_FOR_GOAL'] == 'received_goal':
                return True

            # in all other case, just keep running, don't terminate anything
            return False


        # gets called when ALL child states are terminated
        def out_cb(outcome_map):
            if outcome_map['WAIT_FOR_CHARGE'] == 'charging':
                return 'received_goal'
            if outcome_map['WAIT_FOR_GOAL'] == 'received_goal':
                return 'received_goal'
            else:
                return 'preempted'


        # creating the concurrence state machine
        cc = smach.Concurrence(outcomes = ['received_goal', 'preempted'],
                        output_keys = ['sm_charge', 'sm_pose'],
                        default_outcome = 'preempted',
                        child_termination_cb = child_term_cb,
                        outcome_cb = out_cb)

        cc.userdata.sm_charge = False

        with cc:
            smach.Concurrence.add('WAIT_FOR_GOAL',
                               WaitForGoal(),
                               remapping={'target_pose': 'sm_pose', 'charge': 'sm_charge'})

            smach.Concurrence.add('WAIT_FOR_CHARGE',
                               WaitForCharge(),
                               remapping={'target_pose': 'sm_pose','charge': 'sm_charge'})

        smach.StateMachine.add('WAIT_FOR_COMMAND',
                               cc,
                               transitions={'received_goal': 'NAVIGATE', 'preempted': 'preempted'},
                               remapping= {'sm_pose': 'sm_pose', 'sm_charge': 'sm_charge'})


        smach.StateMachine.add('NAVIGATE',
                               Navigate(),
                               transitions={'succeeded': 'WAIT_FOR_COMMAND', 'preempted': 'WAIT_FOR_COMMAND',
                               'aborted': 'WAIT_FOR_COMMAND','charging': 'NAVIGATE_2_STATION'},
                               remapping={'received_goal': 'sm_pose', 'charge': 'sm_charge'})

        smach.StateMachine.add('NAVIGATE_2_STATION',
                               Navigate2Station(),
                               transitions={'succeeded': 'succeeded',
                               'aborted': 'WAIT_FOR_COMMAND'})


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