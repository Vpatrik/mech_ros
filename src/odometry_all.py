#!/usr/bin/env python

import rospy
import sys

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

imu_orient = Quaternion()
imu_ang_vel = Vector3()
correct_odom = Odometry()


# Define covariances
cov_pose = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2]

cov_twist = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05]


# Define transformation frames
frame_id = "mechROS/tf/odom"
child_frame_id = "/mechROS_base_link"

# Define static information of transformation
odom_tf = TransformStamped()
odom_tf.header.frame_id = frame_id
odom_tf.child_frame_id = child_frame_id

# Define topics
control_odometry_topic = "/mechROS/base_control/odom"
imu_topic = "/imu/data_raw"
publish_odometry_topic = "/mechROS/odom"

def get_yaw(quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

def imu_callback(imu_data):
    global imu_orient, imu_ang_vel
    imu_orient = imu_data.orientation
    imu_ang_vel = imu_data.angular_velocity


def odometry_callback(ctrl_odom):
    global odom_tf, correct_odom, last

    now = rospy.Time.now()
    dt = now.to_sec()-last.to_sec()

    if dt > 0.1:
        last = now
        dt = 0

    # Correct control odometry
    correct_odom.twist.twist.linear = ctrl_odom.twist.twist.linear

    # angular velocity from wheel encoders
    # correct_odom.twist.twist.angular = ctrl_odom.twist.twist.angular

    # angular velocity from imu
    correct_odom.twist.twist.angular = imu_ang_vel

    correct_odom.pose.pose.orientation = imu_orient
    correct_odom.pose.covariance = cov_pose
    correct_odom.twist.covariance = cov_twist

    yaw = get_yaw(imu_orient)
    # create pose from velocity
    correct_odom.pose.pose.position.x = correct_odom.pose.pose.position.x + correct_odom.twist.twist.linear.x*(dt)*math.cos(yaw)
    correct_odom.pose.pose.position.y = correct_odom.pose.pose.position.y + correct_odom.twist.twist.linear.x*(dt)*math.sin(yaw)
    correct_odom.pose.pose.position.z = ctrl_odom.pose.pose.position.z
    last = now

    ## Create transformation
    odom_tf.transform.translation.x = correct_odom.pose.pose.position.x
    odom_tf.transform.translation.y = correct_odom.pose.pose.position.y
    odom_tf.transform.translation.z = correct_odom.pose.pose.position.z
    odom_tf.transform.rotation.x = correct_odom.pose.pose.orientation.x
    odom_tf.transform.rotation.y = correct_odom.pose.pose.orientation.y
    odom_tf.transform.rotation.z = correct_odom.pose.pose.orientation.z
    odom_tf.transform.rotation.w = correct_odom.pose.pose.orientation.w

    ## Publish topic and transformation
    correct_odom.header.stamp = now
    odom_tf.header.stamp = now
    odom_publisher.publish(correct_odom)
    odom_tf_publisher.sendTransform(odom_tf)

if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    last = rospy.Time.now()
    ctrl_odom_subscriber = rospy.Subscriber(control_odometry_topic, Odometry, odometry_callback)
    imu_subscriber = rospy.Subscriber(imu_topic, Imu, imu_callback)
    odom_publisher = rospy.Publisher(publish_odometry_topic, Odometry,queue_size=10)
    odom_tf_publisher = tf2_ros.TransformBroadcaster()
    rospy.spin()
