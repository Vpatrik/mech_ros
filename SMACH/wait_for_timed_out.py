
# Patrik Vavra 2019

import rospy
import smach
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs

class TimedOut(smach.State):


    def __init__(self, time=10):
        smach.State.__init__(self, outcomes=['timed_out', 'preempted', 'aborted'],
                             input_keys=['number_in'],
                             output_keys= ['number_out'])
        self.time = time

    def execute(self, userdata):

        if userdata.number_in > 2:
            rospy.loginfo('Already two unsuccessfull attempts for navigation to charging plug. Aborting!!!')
            return 'aborted'
        rospy.loginfo("Wait for complete navigation to charging station for %d seconds before timing out." % self.time)
        sleeping_step_size = 1 # [s]

        while self.time > 0:
            self.time -= sleeping_step_size
            rospy.sleep(sleeping_step_size)
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('TimedOut state preempted!')
                return 'preempted'
        userdata.number_out = userdata.number_in + 1
        return 'timed_out'