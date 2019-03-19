# Patrik Vavra 2019

import rospy
import smach
import threading
from geometry_msgs.msg import Twist


class PublishVelocity(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['robot_stopped', 'preempted'], input_keys=[], output_keys=[])
        self.velocity = Twist()
        self.publisher = None
        self.message_number = 0
        self.signal = threading.Event()

    def execute(self, userdata):
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.publisher = rospy.Publisher("/station/cmd_vel", Twist, queue_size=3)
        
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            rospy.logdebug("Publishing zero velocity command")
            self.publisher.publish(self.velocity)
            if self.message_number > 2:
                self.signal.set()
            self.message_number += 1
            self.signal.wait(0.2)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return 'preempted'

        rospy.loginfo("Zero velocity command published")

        return 'robot_stopped'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()