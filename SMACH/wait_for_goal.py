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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs

class WaitForGoal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['received_goal', 'preempted'], input_keys=[], output_keys=['target_pose','charge'])
        self.target_pose = PoseStamped()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.signal = threading.Event()
        self.subscriber = None



    def execute(self, user_data):

        rospy.loginfo("Waiting for a goal...")
        self.signal.clear()
        self.subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            rospy.logdebug("Waiting for a goal...")
            self.signal.wait(1)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return 'preempted'

        user_data.target_pose = self.target_pose
        charge = Bool()
        charge.data = False
        user_data.charge = charge
        pos = self.target_pose.pose.position
        rospy.loginfo("Received goal pose: (%s, %s, %s)", pos.x, pos.y, pos.z)

        return 'received_goal'

    def goal_callback(self, msg):
        rospy.logdebug("Received goal pose: %s", str(msg))
        try:
            # Simulated
            trans = self.tf_buffer.lookup_transform('map', 'mechROS_base_link', rospy.Time(0), rospy.Duration(1.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
            pose_transformed.header.frame_id = 'map'
            self.target_pose = pose_transformed

            # Real robot
            # self.target_pose = msg


            self.signal.set()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('Could not find transform')


    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()