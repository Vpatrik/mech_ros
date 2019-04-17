#!/usr/bin/env python

"""
Only for testing computing complete odometry from only IMU
Not used in anywhere else
"""

import rospy
import sys


from mech_ros_msgs.msg import RawImu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math
import yaml



from numpy import linalg, array, sin, cos, multiply

# Define robot parameters
wheel_r = 0.05
L = 0.126 # rozvor
W = 0.147 # rozchod

# Load calibration data
calibrate_file = "/home/patrik/catkin_ws/src/mech_ros/localization/IMU_calibration.yaml"

def load_coefficient(calibration_file):
    with open(calibration_file) as stream:
        try:
            data_loaded = yaml.load(stream)
            acc = data_loaded["Accelerometer"]
            gyro = data_loaded["Gyroscope"]
            M_matrix = array(data_loaded["Matrix_mag"]).reshape((3,3))
            M_bias = array(data_loaded["Matrix_bias"])
            return acc, gyro, M_matrix, M_bias
        except yaml.YAMLError as exc:
            print(exc)


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

orientation_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

ang_vel_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

lin_acc_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

mag_field_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

# Define transformation frames
frame_id = "mechROS/tf/odom"
child_frame_id = "/mechROS_base_link"


# Define topics
raw_imu_topic = "/raw_imu"
publish_imu_odometry_topic = "/IMU/odom"
publish_imu_mag_topic = "imu/mag"
publish_imu_topic = "/imu/data_raw" # For Magdwick filter convension
imu_magdwick_topic = "/imu/data"
velocity_command_topic = "/cmd_vel"

# Po mereni smazat
raw_imu_publish = "raw_data_imu"

# Define static odometry information
odom = Odometry()
odom.header.frame_id = frame_id
odom.child_frame_id = child_frame_id
odom.pose.covariance = cov_pose
odom.twist.covariance = cov_twist


# Define static IMU information
imu = Imu()
imu.header.frame_id = frame_id
imu.orientation_covariance = orientation_cov
imu.angular_velocity_covariance = ang_vel_cov

# Define static magnetic information
mag = MagneticField()
mag.header.frame_id = frame_id
mag.magnetic_field_covariance = mag_field_cov

# Define flags
is_initialized = False
is_calibrated = False
theta_start = 0

# Calibration
samples = 0
acc_bias = [0,0,0]
gyro_bias = [0,0,0]
SamplesNumber = 1000
acc_threshold = 0.2

# Define velocity commands
lin_comm = 0
ang_comm = 0
hysteresis = 0

def get_yaw(quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

def cmd_vel_callback(cmd):
    global lin_comm, ang_comm

    lin_comm = cmd.linear.x
    ang_comm = cmd.angular.z


def imu_madgwick_callback(imu_m):
    global last, odom, is_initialized, hysteresis, theta_start

    now = imu_m.header.stamp
    if not(is_initialized): 
        last = now
        is_initialized = True
        ####### DELETE THETA_START - bylo kvuli stochastice
        theta_start = get_yaw(imu_m.orientation)
        return

    ## Odometry
    odom.header.stamp = now
    dt = now.to_sec() - last.to_sec()
    last = now


    # Pass angular velocity
    odom.twist.twist.angular = imu_m.angular_velocity

    # Pass orientation
    odom.pose.pose.orientation = imu_m.orientation

    # Get yaw information from quaternion from magdwick
    theta = get_yaw(imu_m.orientation) - theta_start

    if abs(lin_comm) > 0.05 or abs(ang_comm) > 0.05 or hysteresis < 10:

        if abs(imu_m.linear_acceleration.x) < 0.05:
            imu_m.linear_acceleration.x = 0

        # # Calculate translational speed - integration - only for 2d
        odom.twist.twist.linear.x +=  dt*imu_m.linear_acceleration.x
        odom.twist.twist.linear.y +=  dt*imu_m.linear_acceleration.y

        # Calculate position
        # odom.pose.pose.position.x += dt*(cos(theta)*odom.twist.twist.linear.x - sin(theta)*odom.twist.twist.linear.y)
        # odom.pose.pose.position.y += dt*(sin(theta)*odom.twist.twist.linear.x + cos(theta)*odom.twist.twist.linear.y)
        odom.pose.pose.position.x += dt*(cos(theta)*odom.twist.twist.linear.x)
        odom.pose.pose.position.y += dt*(sin(theta)*odom.twist.twist.linear.x)
        
        if abs(lin_comm) > 0.05 or abs(ang_comm) > 0.05:
            hysteresis = 0

        else:
            hysteresis+=1

    # Publish odometry topic
    imu_odom_publisher.publish(odom)


def imu_callback(imu_raw):

    global imu, mag, is_calibrated, samples
    now = imu_raw.header.stamp

    ############# Po mereni smazat #####################
    raw_vector6 = Imu()
    raw_vector6.header.stamp = now
    raw_vector6.linear_acceleration.x = imu_raw.raw_linear_acceleration.x
    raw_vector6.linear_acceleration.y = imu_raw.raw_linear_acceleration.y
    raw_vector6.linear_acceleration.z = imu_raw.raw_linear_acceleration.z

    raw_vector6.angular_velocity.x = imu_raw.raw_angular_velocity.x
    raw_vector6.angular_velocity.y = imu_raw.raw_angular_velocity.y
    raw_vector6.angular_velocity.z = imu_raw.raw_angular_velocity.z
    imu_raw_publisher.publish(raw_vector6)
    #######################################################

    if not(is_calibrated):
        acc_bias[0] = acc_bias[0] + imu_raw.raw_linear_acceleration.x
        acc_bias[1] = acc_bias[1] +imu_raw.raw_linear_acceleration.y
        acc_bias[2] = acc_bias[2] + imu_raw.raw_linear_acceleration.z

        gyro_bias[0] = gyro_bias[0] + imu_raw.raw_angular_velocity.x
        gyro_bias[1] = gyro_bias[1] + imu_raw.raw_angular_velocity.y
        gyro_bias[2] = gyro_bias[2] + imu_raw.raw_angular_velocity.z
        samples+=1



        if samples >= SamplesNumber:
            is_calibrated = True
            acc_bias[0] = acc_bias[0]/SamplesNumber
            acc_bias[1] = acc_bias[1]/SamplesNumber
            acc_bias[2] = acc_bias[2]/SamplesNumber

            gyro_bias[0] = gyro_bias[0]/SamplesNumber
            gyro_bias[1] = gyro_bias[1]/SamplesNumber
            gyro_bias[2] = gyro_bias[2]/SamplesNumber
            rospy.loginfo('IMU calibrated')
        return



    ## Correct imu data with calibration value
    imu.header.stamp = now
    # Accelerometer
    imu.linear_acceleration.x = imu_raw.raw_linear_acceleration.x - acc_bias[0]
    imu.linear_acceleration.y = imu_raw.raw_linear_acceleration.y - acc_bias[1]
    imu.linear_acceleration.z = imu_raw.raw_linear_acceleration.z - acc_bias[2] + 9.81

    # Gyroscope
    imu.angular_velocity.x = imu_raw.raw_angular_velocity.x - gyro_bias[0]
    imu.angular_velocity.y = imu_raw.raw_angular_velocity.y - gyro_bias[1]
    imu.angular_velocity.z = imu_raw.raw_angular_velocity.z - gyro_bias[2]

    # Magnetic
    mag_raw = array([imu_raw.raw_magnetic_field.x,imu_raw.raw_magnetic_field.y,imu_raw.raw_magnetic_field.z])
    mag_raw = multiply(mag_raw,1e-6) # Convert from microTesla to Tesla
    mag_cal = M_matrix.dot((mag_raw-M_bias.T).T).T
    mag.magnetic_field.x = mag_cal[0]
    mag.magnetic_field.y = mag_cal[1]
    mag.magnetic_field.z = mag_cal[2]
    mag.header.stamp = now

    ## Publish topics
    imu_publisher.publish(imu)
    imu_mag_publisher.publish(mag)



if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    acc, gyro, M_matrix, M_bias = load_coefficient(calibrate_file)
    last = rospy.Time.now()
    imu_raw_subscriber = rospy.Subscriber(raw_imu_topic, RawImu, imu_callback)
    cmd_vel_subscriber = rospy.Subscriber(velocity_command_topic, Twist, cmd_vel_callback)
    imu_magdwick_subscriber = rospy.Subscriber(imu_magdwick_topic, Imu, imu_madgwick_callback)
    imu_odom_publisher = rospy.Publisher(publish_imu_odometry_topic, Odometry,queue_size=10)
    imu_mag_publisher = rospy.Publisher(publish_imu_mag_topic, MagneticField,queue_size=10)
    imu_publisher = rospy.Publisher(publish_imu_topic, Imu,queue_size=10)

    # Po mereni smazat
    imu_raw_publisher = rospy.Publisher(raw_imu_publish, Imu,queue_size=10)
    rospy.spin()
