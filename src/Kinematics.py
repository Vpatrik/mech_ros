#!/usr/bin/env python

# Node which receive raw wheels velocities data from rosserial_python node from arduino, transform data in
# physical meaning and compute odometry message for Extended Kalman filter

import rospy
import sys


from mech_ros_msgs.msg import WheelsVelocities
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from math import cos, sin


class Kinematics:
    def __init__(self):
        rospy.init_node("kinematics")


        self.last_time = rospy.Time.now()
        self.flag_initialized = False

        # ROS adjustable parameters
        frame_id = rospy.get_param("~map_frame_id", "map")
        child_frame_id = rospy.get_param("~odom_frame_id", "mechROS_base_link")
        wheels_vel_topic = rospy.get_param("~wheels_vel_topic", "/wheels_vel")
        odom_topic = rospy.get_param("~odom_topic", "/mechROS/odom")
        self.radius = rospy.get_param("~wheels_radius", 0.045608)
        self.width = rospy.get_param("~base_width", 0.19007)
        self.a = rospy.get_param("~a_kinematics_param", 0.26484)
        self.b = rospy.get_param("~b_kinematics_param", 0.10107)
        
        # Covariances - maybe would be better to add them as adjustable parameters
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

        # Intialize message
        self.Odom = Odometry()
        self.Odom.header.frame_id = frame_id
        self.Odom.child_frame_id = child_frame_id
        self.Odom.pose.covariance = cov_pose
        self.Odom.twist.covariance = cov_twist

        # ROS Subscribers and publishers
        rospy.Subscriber(wheels_vel_topic, Odometry, self.wheels_vel_cb)
        self.odom_publisher = rospy.Publisher(odom_topic, Odometry, queue_size=10)

                

    def wheels_vel_cb(self, velocities):
        now = rospy.Time.now()

        if not self.flag_initialized:
            rospy.loginfo("First wheels velocities information received!")
            self.last_time = now
            self.flag_initialized = True
            return

        dt = now.to_sec()-self.last_time.to_sec()
        self.last_time = now

        # Calculate average speed of side
        speed_r = (velocities.front_right + velocities.rear_right)/2
        speed_l = (velocities.front_left + velocities.rear_left)/2

        
        # Calculate translational and angular speed in robot frame
        x_r_dot = self.radius*(speed_l + speed_r)/2

        if abs(speed_l - speed_r) < 1e-9:
            Lambda = (speed_l + speed_r)/(1e-9)
        else:
            Lambda = (speed_l + speed_r)/(speed_r - speed_l)

        chi = 1 + self.a/(1 + self.b*abs(Lambda)**0.5)  
        theta_dot = self.radius*(speed_r - speed_l)/(chi*self.width)

        # Fill twist part of Odometry message
        self.Odom.twist.twist.linear.x = x_r_dot
        self.Odom.twist.twist.angular.z = theta_dot
        self.Odom.header.stamp = now

        # Calculate pose
        yaw = self.getYaw(self.Odom.pose.pose.orientation)
        self.Odom.pose.pose.position.x += dt*x_r_dot*cos(yaw)
        self.Odom.pose.pose.position.y += dt*x_r_dot*sin(yaw)
        yaw += dt*theta_dot
        quat = quaternion_from_euler(0,0,yaw)
        self.Odom.pose.pose.orientation.x = quat[0]
        self.Odom.pose.pose.orientation.y = quat[1]
        self.Odom.pose.pose.orientation.z = quat[2]
        self.Odom.pose.pose.orientation.w = quat[3]

        ## Publish Odometry message
        self.odom_publisher.publish(self.Odom)

    def getYaw(self, quaternion):
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw


if __name__ == '__main__':
    try:
        SkidSteerKinematics = Kinematics()
    except rospy.ROSInterruptException:
        pass
