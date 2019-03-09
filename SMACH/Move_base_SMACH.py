#!/usr/bin/env python

import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction

import tf2_ros
import tf2_geometry_msgs


@smach.cb_interface(input_keys=['target_pose'])
def get_path_goal_cb(userdata, goal):
    goal.use_start_pose = False
    goal.tolerance = 0.2
    goal.target_pose = userdata.target_pose
    goal.planner = 'planner'



def main():
    rospy.init_node('mbf_state_machine')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.goal = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    with sm:
        # Goal callback for state WAIT_FOR_GOAL
        def goal_cb(userdata, msg):
            try:
                trans = tf_buffer.lookup_transform('map', 'mechROS_base_link', rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
                pose_transformed.header.frame_id = 'map'
                userdata.goal = pose_transformed
                return False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo('Could not find transform')

            return True

        # Monitor topic to get MeshGoal from RViz plugin
        smach.StateMachine.add(
            'WAIT_FOR_GOAL',
            smach_ros.MonitorState(
                '/move_base_simple/goal',
                PoseStamped,
                goal_cb,
                output_keys=['goal']
            ),
            transitions={
                'invalid': 'GET_PATH',
                'valid': 'WAIT_FOR_GOAL',
                'preempted': 'preempted'
            }
        )

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/get_path',
                GetPathAction,
                goal_slots=['target_pose'],
                result_slots=['path']
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'WAIT_FOR_GOAL',
                'preempted': 'preempted'
            },
            remapping={
                'target_pose': 'goal'
            }
        )

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'WAIT_FOR_GOAL',
                'aborted': 'RECOVERY',
                'preempted': 'preempted'
            }
        )

        # Goal callback for state RECOVERY
        def recovery_path_goal_cb(userdata, goal):
            if userdata.clear_costmap_flag == False:
                goal.behavior = 'clear_costmap'
                userdata.clear_costmap_flag = True
            else:
                goal.behavior = 'straf_recovery'
                userdata.clear_costmap_flag = False

        # Recovery
        smach.StateMachine.add(
            'RECOVERY',
             smach_ros.SimpleActionState(
                'move_base_flex/recovery',
                RecoveryAction,
                goal_cb=recovery_path_goal_cb,
                input_keys=["error", "clear_costmap_flag"],
                output_keys = ["error_status", 'clear_costmap_flag']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    sm.execute()

    # Wait for interrupt and stop introspection server
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()