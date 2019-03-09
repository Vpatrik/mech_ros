#!/usr/bin/env python2

# Taken from https://github.com/juancamilog/calibrate_imu.git and adjusuted for custom needs
# Not specified license in github repo - Hope it's OK to use the code ??? 

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Empty, EmptyResponse
import math
import pickle
from time import localtime,strftime
import os
from numpy import linalg,array,amax,amin,add,divide,multiply,subtract,power,Inf,matrix,tile,bmat,random,arange,ones,zeros,eye,squeeze, cov, mean


class calibrate_imu:
    def __init__(self):
        self.mag_samples = []
        self.acc_samples = []
        self.gyr_samples = []
        self.mag_matrix = eye(3)
        self.mag_offset = zeros(3)
        self.sampling = False
        self.sampling_imu = False

        mag_topic = rospy.get_param('~mag_topic', '/imu/mag')
        imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.publish_calibrated = rospy.get_param("~publish_calibrated", False)
        self.max_samples = rospy.get_param("~max_samples_mag", 15000)
        self.max_samples_imu = rospy.get_param("~max_samples_imu", 1000000)
        self.__location__ = rospy.get_param("~calibrations_dir", "/home/patrik/catkin_ws/src/calibrate_imu/mag_calibration")
        self.calibrated_mag_pub = rospy.Publisher('/imu/mag_calibrated',Vector3Stamped, queue_size='1')
        self.gravity = rospy.get_param("~gravity", 9.81)
        # self.load_calibration()

        rospy.Subscriber(mag_topic,Vector3Stamped,self.mag_cb)
        rospy.Subscriber(imu_topic,Imu, self.imu_cb)
        rospy.Service('start_sampling_magnet',Empty,self.start_sampling)
        rospy.Service('stop_sampling_magnet',Empty,self.stop_sampling)
        rospy.Service('calibrate_magnet',Empty,self.calibrate_mag)

        rospy.Service('start_sampling_imu',Empty,self.start_sampling_imu)
        rospy.Service('stop_sampling_imu',Empty,self.stop_sampling_imu)
        rospy.Service('calibrate_imu',Empty,self.calibrate_imu)

    def load_calibration(self):
        file_path = os.path.join(self.__location__,"last_mag_calibration")
        try:
            f = open(file_path)
            self.mag_offset = squeeze(array(pickle.load(f)))
            self.mag_matrix = pickle.load(f)
            self.mag_samples = pickle.load(f)
            f.close()
        except:
            rospy.log_error("Could not open file %s"%(file_path))

        rospy.loginfo("Magnetometer offset:\n %s",self.mag_offset)
        rospy.loginfo("Magnetometer Calibration Matrix:\n %s",self.mag_matrix)

    # callback for magnetometer readings
    def mag_cb(self,data):
        if self.sampling == True:
             #collect measurements
             self.mag_samples.append([data.vector.x,data.vector.y,data.vector.z])
             if len(self.mag_samples)%50==0:
                 rospy.loginfo("Got %d magnetometer readings"%(len(self.mag_samples)))
             if len(self.mag_samples) > self.max_samples:
                 # drop the 50 oldest samples
                 self.mag_samples = self.mag_samples[50:]

        if self.publish_calibrated == True:
            mag_raw = array([data.vector.x,data.vector.y,data.vector.z])
            mag_cal = ((self.mag_matrix.dot((mag_raw-self.mag_offset.T).T)).T)
            calibrated_mag_msg = Vector3Stamped()
            calibrated_mag_msg.header.stamp = data.header.stamp
            calibrated_mag_msg.vector.x = mag_cal[0]
            calibrated_mag_msg.vector.y = mag_cal[1]
            calibrated_mag_msg.vector.z = mag_cal[2]

            self.calibrated_mag_pub.publish(calibrated_mag_msg)

    def imu_cb(self,data):
        if self.sampling_imu == True:
            #collect measurements
            self.acc_samples.append([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
            self.gyr_samples.append([data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z])
            if len(self.acc_samples)%50==0:
                rospy.loginfo("Got %d imu readings"%(len(self.acc_samples)))
            if len(self.acc_samples) > self.max_samples_imu:
                # drop the 50 oldest samples
                self.acc_samples = self.acc_samples[50:]
                self.gyr_samples = self.gyr_samples[50:]

    def start_sampling(self,req):
        rospy.loginfo('Started collecting magnetometer readings')
        self.sampling = True
        self.mag_samples = []
        return EmptyResponse()

    def stop_sampling(self,req):
        rospy.loginfo('Stopped collecting magnetometer readings')
        self.sampling = False
        return EmptyResponse()

    def start_sampling_imu(self,req):
        rospy.loginfo('Started collecting imu readings')
        self.sampling_imu = True
        self.imu_samples = []
        return EmptyResponse()

    def stop_sampling_imu(self,req):
        rospy.loginfo('Stopped collecting imu readings')
        self.sampling_imu = False
        return EmptyResponse()


    def calibrate_mag(self,req):
        if len(self.mag_samples)>=10:
            self.sampling = False
            xyz = matrix(self.mag_samples)
            rospy.loginfo('Starting magnetometer calibration with %d samples'%(len(self.mag_samples)))

            #compute the vectors [ x^2 y^2 z^2 2*x*y 2*y*z 2*x*z x y z 1] for every sample
            # the result for the x*y y*z and x*z components should be divided by 2
            xyz2 = power(xyz,2)
            xy = multiply(xyz[:,0],xyz[:,1])
            xz = multiply(xyz[:,0],xyz[:,2])
            yz = multiply(xyz[:,1],xyz[:,2])

            # build the data matrix
            A = bmat('xyz2 xy xz yz xyz')

            b = 1.0*ones((xyz.shape[0],1))

            # solve the system Ax = b
            q,res,rank,sing = linalg.lstsq(A,b)

            # build scaled ellipsoid quadric matrix (in homogeneous coordinates)
            A = matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0],0.5*q[6][0]],
                        [0.5*q[3][0],q[1][0],0.5*q[5][0],0.5*q[7][0]],
                        [0.5*q[4][0],0.5*q[5][0],q[2][0],0.5*q[8][0]],
                        [0.5*q[6][0],0.5*q[7][0],0.5*q[8][0],-1]])

            # build scaled ellipsoid quadric matrix (in regular coordinates)
            Q = matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0]],
                        [0.5*q[3][0],q[1][0],0.5*q[5][0]],
                        [0.5*q[4][0],0.5*q[5][0],q[2][0]]])

            # obtain the centroid of the ellipsoid
            x0 = linalg.inv(-1.0*Q)*matrix([0.5*q[6][0],0.5*q[7][0],0.5*q[8][0]]).T

            # translate the ellipsoid in homogeneous coordinates to the center
            T_x0 = matrix(eye(4))
            T_x0[0,3] = x0[0]; T_x0[1,3] = x0[1]; T_x0[2,3] = x0[2]
            A = T_x0.T*A*T_x0

            # rescale the ellipsoid quadric matrix (in regular coordinates)
            Q = Q*(-1.0/A[3,3])

            # take the cholesky decomposition of Q. this will be the matrix to transform
            # points from the ellipsoid to a sphere, after correcting for the offset x0
            L = eye(3)
            try:
                L = linalg.cholesky(Q).transpose()
            except Exception,e:
                rospy.loginfo(str(e))
                L = eye(3)

            rospy.loginfo("Magnetometer offset:\n %s",x0)
            rospy.loginfo("Magnetometer Calibration Matrix:\n %s",L)

            # file_path = os.path.join(self.__location__,"last_mag_calibration")
            file_path = '/home/patrik/catkin_ws/src/mech_ros/localization/last_mag_calibration'
            f = open(file_path,"w+")
            pickle.dump(x0,f)
            pickle.dump(L,f)
            pickle.dump(matrix(self.mag_samples),f)
            f.close()
            # back up calibration
            calib_name = strftime("mag_calibration_%d_%m_%y_%H_%M_%S",localtime())
            calib_path = os.path.join('/home/patrik/catkin_ws/src/mech_ros/localization/zaloha',calib_name)
            # calib_path = '/home/patrik/catkin_ws/src/mech_ros/localization/zaloha'
            os.system("cp %s %s"%(file_path,calib_path))

            self.mag_matrix = L
            self.mag_offset = squeeze(array(x0))
        else:
            rospy.loginfo("Not enough samples to calibrate the magnetometer")
        return EmptyResponse()

    def calibrate_imu(self,req):
        if len(self.acc_samples)>=100:
            self.sampling_imu = False
            acc = array(self.acc_samples)
            gyr = array(self.gyr_samples)
            rospy.loginfo('Starting imu calibration with %d samples'%(len(self.acc_samples)))


            try:
                acc_cov = cov(acc,rowvar = False)
                acc_mean = mean(acc, axis=0)
                acc_mean[2] -= self.gravity
                gyr_cov = cov(gyr,rowvar = False)
                gyr_mean = mean(gyr, axis=0)

            except Exception,e:
                rospy.loginfo(str(e))

            rospy.loginfo("Accelerometer bias offset:\n %s",acc_mean)
            rospy.loginfo("Accelerometer covariance matrix:\n %s",acc_cov)

            rospy.loginfo("Gyro bias offset:\n %s",gyr_mean)
            rospy.loginfo("Gyro covariance matrix:\n %s",gyr_cov)

            # file_path = os.path.join(self.__location__,"last_mag_calibration")
            file_path = '/home/patrik/catkin_ws/src/mech_ros/localization/last_imu_calibration'
            f = open(file_path,"w+")
            pickle.dump(acc_mean,f)
            pickle.dump(acc_cov,f)

            pickle.dump(gyr_mean,f)
            pickle.dump(gyr_cov,f)
            f.close()

            # back up calibration
            calib_name = strftime("imu_calibration_%d_%m_%y_%H_%M_%S",localtime())
            calib_path = os.path.join('/home/patrik/catkin_ws/src/mech_ros/localization/zaloha',calib_name)
            # calib_path = '/home/patrik/catkin_ws/src/mech_ros/localization/zaloha'
            os.system("cp %s %s"%(file_path,calib_path))

        else:
            rospy.loginfo("Not enough samples to calibrate imu")
        return EmptyResponse()
   

if __name__=='__main__':
    rospy.init_node('calibrate_imu')
    cbimu = calibrate_imu()
    rospy.spin()


