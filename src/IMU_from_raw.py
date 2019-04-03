#!/usr/bin/env python

import rospy
import sys


from mech_ros_msgs.msg import RawImu
from mech_ros_msgs.srv import Start_recalibration
from mech_ros_msgs.srv import Start_recalibrationRequest
from mech_ros_msgs.srv import Start_recalibrationResponse

from mech_ros_msgs.srv import End_recalibration
from mech_ros_msgs.srv import End_recalibrationRequest
from mech_ros_msgs.srv import End_recalibrationResponse

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from math import pi
import yaml

from numpy import linalg, array, sin, cos, multiply, dot, zeros, mean

# Node which receives raw data from rosserial_python node from arduino, transform data in
# physical meaning and fill type of messages to use default Madgwick node
# Calibration on Start
# Recalibration as Service - /start_recalibraion, /stop_recalibration
# TODO - implement velocity lock when calibrating imu

class ImuMagConverter:
    def __init__(self):
        rospy.init_node("imu_mag_converter")

        self.acc_samples = array([0,0,0])
        self.gyr_samples = array([0,0,0])
        # self.acc_samples = []
        # self.gyr_samples = []
        self.acc_bias = zeros([1,3])
        self.gyro_bias = zeros([1,3])
        self.acc_diag_cov = zeros([1,3])
        self.gyr_diag_cov = zeros([1,3])
        self.mag_bias = zeros([1,3])
        self.mag_matrix = zeros([3,3])
        self.calibration_number = 0
        self.recalibrate_imu = False

        # IMU and Mag parameters
        self.a_gain = 0.061/1000
        self.gravity = 9.81
        self.m_gain = 0.14/1000
        self.g_gain = 8.75*pi/(1000*180)


        # ROS adjustable parameters
        imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        mag_frame_id = rospy.get_param("~mag_frame_id", "magnetic_link")
        raw_imu_mag_topic = rospy.get_param("~subscribe_raw_imu_topic", "/raw_imu")
        mag_topic = rospy.get_param("~publish_mag_topic", "/imu/mag")
        imu_topic = rospy.get_param("~publish_imu_topic", "/imu/data_raw")
        self.calibrate_imu = rospy.get_param("~calibrate_imu", False)
        self.samples_number = rospy.get_param("~samples_number", 1000)
        self.calibration_file = rospy.get_param("~calibration_file", "/home/patrik/catkin_ws/src/mech_ros/localization/IMU_calibration.yaml")
        
        # Intialize message
        self.Imu = Imu()
        self.Imu.header.frame_id = imu_frame_id
        self.Imu.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.Imu.angular_velocity_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self.Imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        self.Mag = Vector3Stamped()
        self.Mag.header.frame_id = mag_frame_id

        # ROS Subscribers, publishers and services
        rospy.Subscriber(raw_imu_mag_topic,RawImu, self.raw_imu_mag_cb)
        self.imu_publisher = rospy.Publisher(imu_topic, Imu, queue_size=10)
        self.mag_publisher = rospy.Publisher(mag_topic, Vector3Stamped, queue_size=10)
        rospy.Service('start_recalibration',Start_recalibration, self.start_imu_recalibration)
        rospy.Service('stop_recalibration',End_recalibration, self.stop_imu_recalibration)

        # Load calibration data
        self.load_calibration()

    def load_calibration(self):
        with open(self.calibration_file) as stream:
            try:
                data_loaded = yaml.load(stream)
                self.acc_bias = data_loaded["Acc_bias"]
                self.acc_diag_cov = data_loaded["Acc_covariance"]
                self.gyr_bias = data_loaded["Gyr_bias"]
                self.gyr_diag_cov = data_loaded["Gyr_covariance"]
                self.mag_matrix = array(data_loaded["Mag_matrix"]).reshape((3,3))
                self.mag_bias = array(data_loaded["Mag_bias"])

                self.Imu.linear_acceleration_covariance[0] = self.acc_diag_cov[0]
                self.Imu.linear_acceleration_covariance[3] = self.acc_diag_cov[1]
                self.Imu.linear_acceleration_covariance[6] = self.acc_diag_cov[2]

                self.Imu.angular_velocity_covariance[0] = self.gyr_diag_cov[0]
                self.Imu.angular_velocity_covariance[3] = self.gyr_diag_cov[1]
                self.Imu.angular_velocity_covariance[6] = self.gyr_diag_cov[2]

            except yaml.YAMLError as exc:
                rospy.log_error("Could not open file %s"%(self.calibration_file))
                rospy.log_error(exc)
                

    def raw_imu_mag_cb(self, imu_raw):
        now = rospy.Time.now()

        # If chosen calibrate IMU
        # if self.calibrate_imu:
        #     self.acc_samples.append([imu_raw.raw_linear_acceleration.x, imu_raw.raw_linear_acceleration.y,imu_raw.raw_linear_acceleration.z])
        #     self.gyr_samples.append([imu_raw.raw_angular_velocity.x,imu_raw.raw_angular_velocity.y,imu_raw.raw_angular_velocity.z])

        #     if len(self.acc_samples) >= self.samples_number:
        #         self.calibrate_imu = False
        #         self.acc_bias = mean(array(self.acc_samples)*self.a_gain*self.gravity,axis=0)
        #         self.gyr_bias = mean(array(self.gyr_samples)*self.g_gain,axis=0)

        #         # Publish info about calibration in ROS 
        #         rospy.loginfo('IMU calibrated from %d samples'%(len(self.acc_samples)))
        #         rospy.loginfo("Accelerometer bias :\n %s",self.acc_bias)
        #         rospy.loginfo("Gyroscope bias :\n %s",self.gyr_bias)
        #     return

        if self.calibrate_imu:
            self.acc_samples[0] += imu_raw.raw_linear_acceleration.x
            self.acc_samples[1] += imu_raw.raw_linear_acceleration.y
            self.acc_samples[2] += imu_raw.raw_linear_acceleration.z

            self.gyr_samples[0] += imu_raw.raw_angular_velocity.x
            self.gyr_samples[1] += imu_raw.raw_angular_velocity.y
            self.gyr_samples[2] += imu_raw.raw_angular_velocity.z

            self.calibration_number +=1

            if (self.calibration_number >= self.samples_number) and not(self.recalibrate_imu):
                self.calibrate_imu = False
                self.recalibrate()

            if not(self.recalibrate_imu):
                return

        ## Multiply each sensor analog value by gain and correct from calibration data
        # Accelerometer
        self.Imu.linear_acceleration.x = -imu_raw.raw_linear_acceleration.x*self.a_gain*self.gravity + self.acc_bias[0]
        self.Imu.linear_acceleration.y = imu_raw.raw_linear_acceleration.y*self.a_gain*self.gravity - self.acc_bias[1]
        self.Imu.linear_acceleration.z = imu_raw.raw_linear_acceleration.z*self.a_gain*self.gravity - self.acc_bias[2]

        # Gyroscope
        self.Imu.angular_velocity.x = -imu_raw.raw_angular_velocity.x*self.g_gain + self.gyr_bias[0]
        self.Imu.angular_velocity.y = imu_raw.raw_angular_velocity.y*self.g_gain - self.gyr_bias[1]
        self.Imu.angular_velocity.z = imu_raw.raw_angular_velocity.z*self.g_gain - self.gyr_bias[2]

        self.Imu.header.stamp = now

        # Magnetometer
        mag_raw = array([imu_raw.raw_magnetic_field.x,imu_raw.raw_magnetic_field.y,imu_raw.raw_magnetic_field.z])
        mag_raw *= self.m_gain
        mag_recal = self.mag_matrix.dot((mag_raw-self.mag_bias).T).T

        mag_recal = mag_raw
        self.Mag.vector.x = mag_recal[0]
        self.Mag.vector.y = mag_recal[1]
        self.Mag.vector.z = mag_recal[2]
        
        self.Mag.header.stamp = now

        ## Publish topics
        self.imu_publisher.publish(self.Imu)
        self.mag_publisher.publish(self.Mag)

    def start_imu_recalibration(self,req):
        # TODO - implement some logic to reject recalibration if not convinient

        # Initialize
        self.acc_samples = array([0,0,0])
        self.gyr_samples = array([0,0,0])
        self.calibration_number = 0
        self.calibrate_imu = True
        self.recalibrate_imu = True

        response = Start_recalibrationResponse()
        response.accepted = True

        rospy.loginfo('Started collecting imu readings for recalibration')
        # TODO - Implement only partial recalibration
        self.recalibrate_gyr = req.calibrate_gyr
        self.recalibrate_gyr = req.calibrate_acc

        return response
    
    def stop_imu_recalibration(self,req):
        rospy.loginfo('Stopped collecting imu readings')
        self.calibrate_imu = False
        self.recalibrate_imu = False
        self.recalibrate()

        response = End_recalibrationResponse()
        response.imu_samples = self.calibration_number

        response.acc_bias.x = self.acc_bias[0]
        response.acc_bias.y = self.acc_bias[1]
        response.acc_bias.z = self.acc_bias[2]

        response.gyr_bias.x = self.gyr_bias[0]
        response.gyr_bias.y = self.gyr_bias[1]
        response.gyr_bias.z = self.gyr_bias[2]
        return response

    def recalibrate(self):
        self.acc_bias = self.acc_samples*self.a_gain*self.gravity/self.calibration_number
        self.acc_bias[2] -= 9.81
        self.gyr_bias = self.gyr_samples*self.g_gain/self.calibration_number

        # Publish info about calibration in ROS 
        rospy.loginfo('IMU calibrated from %d samples'%(self.calibration_number))
        rospy.loginfo("Accelerometer bias :\n %s",self.acc_bias)
        rospy.loginfo("Gyroscope bias :\n %s",self.gyr_bias)


if __name__ == '__main__':
    try:
        imu_mag_converter = ImuMagConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
