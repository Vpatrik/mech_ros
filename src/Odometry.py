#!/usr/bin/env python

import rospy

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped

imu_orient = Quaternion()
imu_ang_vel = Vector3()

frame_id = "mechROS/tf/odom"
child_frame_id = "/mechROS_base_link"

# Define static information of transformation
odom_tf = TransformStamped()
odom_tf.header.frame_id = frame_id
odom_tf.child_frame_id = child_frame_id

# Define topics
control_odometry_topic = "/mechROS/base_control/odom"
imu_topic = "/imu/data"
publish_odometry_topic = "/mechROS/odom"


def imu_callback(imu_data):
    global imu_orient, imu_ang_vel
    imu_orient = imu_data.orientation
    imu_ang_vel = imu_data.angular_velocity


def odometry_callback(ctrl_odom):
    global odom_tf
    now = rospy.Time.now()
    # Change orientation from /control/odom
    ctrl_odom.pose.pose.orientation = imu_orient
    # ctrl_odom.twist.twist.angular = imu_ang_vel

    ## Create transformation
    odom_tf.transform.translation.x = ctrl_odom.pose.pose.position.x
    odom_tf.transform.translation.y = ctrl_odom.pose.pose.position.y
    odom_tf.transform.translation.z = ctrl_odom.pose.pose.position.z
    odom_tf.transform.rotation.x = ctrl_odom.pose.pose.orientation.x
    odom_tf.transform.rotation.y = ctrl_odom.pose.pose.orientation.y
    odom_tf.transform.rotation.z = ctrl_odom.pose.pose.orientation.z
    odom_tf.transform.rotation.w = ctrl_odom.pose.pose.orientation.w

    ## Publish topic and transformation
    ctrl_odom.header.stamp = now
    odom_tf.header.stamp = now
    odom_publisher.publish(ctrl_odom)
    odom_tf_publisher.sendTransform(odom_tf)

if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    ctrl_odom_subscriber = rospy.Subscriber(control_odometry_topic, Odometry, odometry_callback)
    imu_subscriber = rospy.Subscriber(imu_topic, Imu, imu_callback)
    odom_publisher = rospy.Publisher(publish_odometry_topic, Odometry,queue_size=10)
    odom_tf_publisher = tf2_ros.TransformBroadcaster()
    rospy.spin()
