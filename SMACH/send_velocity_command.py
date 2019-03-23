# Patrik Vavra 2019

import rospy
import smach
import threading
from geometry_msgs.msg import Twist


class PublishVelocity(smach.State):

    def __init__(self, cmd_vel):
        smach.State.__init__(self, outcomes=['velocity_published', 'preempted'], input_keys=[], output_keys=[])
        self.velocity = cmd_vel
        self.publisher = None
        self.message_number = 0
        self.signal = threading.Event()

    def execute(self, userdata):
        self.publisher = rospy.Publisher("/station/cmd_vel", Twist, queue_size=3)
        
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            rospy.logdebug("Publishing velocity command")
            self.publisher.publish(self.velocity)
            if self.message_number > 2:
                self.signal.set()
            self.message_number += 1
            self.signal.wait(0.1)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return 'preempted'

        rospy.loginfo("Velocity command published")

        return 'velocity_published'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()