#!/usr/bin/env python
# For testing odometry calculation from only Wheel encoders

import rospy
import sys


from mech_ros_msgs.msg import WheelsVelocities
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from math import cos, sin


# Estimated parameters from measurement - Matlab
radius = 0.045608
width = 0.19007
a = 0.26484
b = 0.10107


# Define covariances
cov_pose = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2]

cov_twist = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.005, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08]

# Define transformation frames
frame_id = "map"
child_frame_id = "mechROS_base_link"


# Define topics
wheels_velocities_topic = "/wheels_vel"
publish_odometry_topic = "/mechROS/odom"

# Define static odometry information
odom = Odometry()
odom.header.frame_id = frame_id
odom.child_frame_id = child_frame_id
odom.pose.covariance = cov_pose
odom.twist.covariance = cov_twist

def getYaw(quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw


def wheels_vel_callback(velocites):

    # global odom, x, y, theta, is_initialized, last
    global odom, last
    now = rospy.Time.now()
    dt = now.to_sec()-last.to_sec()

    if dt > 0.1:
        last = now
        dt = 0
        return
        
    last = now

    # Calculate average speed of side
    speed_r = (velocites.front_right + velocites.rear_right)/2
    speed_l = (velocites.front_left + velocites.rear_left)/2

    
    # Calculate translational and angular speed in robot frame
    x_r_dot = radius*(speed_l + speed_r)/2

    if abs(speed_l - speed_r) < 1e-9:
        Lambda = (speed_l + speed_r)/(1e-9)
    else:
        Lambda = (speed_l + speed_r)/(speed_r - speed_l)

    chi = 1 + a/(1 + b*abs(Lambda)**0.5)  
    theta_dot = radius*(speed_r - speed_l)/(chi*width)

    odom.twist.twist.linear.x = x_r_dot
    odom.twist.twist.angular.z = theta_dot
    odom.header.stamp = now
    odom.header.frame_id = frame_id


    # Calculate pose
    yaw = getYaw(odom.pose.pose.orientation)
    odom.pose.pose.position.x += dt*x_r_dot*cos(yaw)
    odom.pose.pose.position.y += dt*x_r_dot*sin(yaw)
    yaw += dt*theta_dot
    quat = quaternion_from_euler(0,0,yaw)
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]
    ## Publish topic
    odom_publisher.publish(odom)


if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    last = rospy.Time.now()
    wheels_vel_subscriber = rospy.Subscriber(wheels_velocities_topic, WheelsVelocities, wheels_vel_callback)
    odom_publisher = rospy.Publisher(publish_odometry_topic, Odometry,queue_size=10)
    rospy.spin()
