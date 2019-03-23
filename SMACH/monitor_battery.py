# Copyright (c) 2018, Sebastian Putz
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

import rospy
import smach
import threading
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

class WaitForCharge(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['charging', 'preempted'], input_keys=[], output_keys=['charge', 'target_pose'])
        self.charge = Bool()
        self.signal = threading.Event()
        self.subscriber = None


    def execute(self, user_data):

        rospy.loginfo("Waiting for a low battery voltage...")
        self.signal.clear()
        self.subscriber = rospy.Subscriber('/volt_battery', Int8, self.volt_callback)
        
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            rospy.logdebug("Monitoring battery state...")
            self.signal.wait(1)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return 'preempted'

        user_data.charge = self.charge
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        user_data.target_pose = target_pose
        rospy.loginfo("Aborting current state, starting to navigate to charging station!")

        return 'charging'

    def volt_callback(self, msg):
        raw_volt_level = msg.data
        volt_level = (raw_volt_level + 335.0) / 100.0

        if volt_level < 3.5:
            rospy.loginfo("Low battery voltage!")
            self.charge.data = True
            self.signal.set()


    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()