#!/usr/bin/env python

# Control of real robot
import rospy
import sys


from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

# Define robot parameters
wheel_r = 0.048
L = 0.126 # rozvor
W = 0.15 # rozchod

# Define scales
l_scale = -0.22
a_scale = 3.5
# Define which axes to map
linear = 0
angular = 1

# Define topics
publish_twist_topic = "/joystick/cmd_vel"
joy_topic = "/joy"


def joy_callback(Joy_m):
    Robot_twist = Twist()

    Robot_twist.linear.x = l_scale*Joy_m.axes[linear]
    Robot_twist.angular.z = a_scale*Joy_m.axes[angular]

    # Publish twist
    twist_publisher.publish(Robot_twist)

if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    joy_subscriber = rospy.Subscriber(joy_topic, Joy, joy_callback)
    twist_publisher = rospy.Publisher(publish_twist_topic,Twist,queue_size=10)
    rospy.spin()