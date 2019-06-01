#!/usr/bin/env python

# Patrik Vavra 2019

from math import atan2, cos, pi, sin
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import (PoseWithCovarianceStamped, Quaternion,
                               Transform, TransformStamped)
from mech_ros_msgs.msg import Marker, MarkerList
from numpy import dot, linalg, where, zeros, ones, array, multiply
from numpy import sum as sum_n
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from scipy import linalg as lin

def inversion(m):    
    m1, m2, m3, m4, m5, m6, m7, m8, m9 = m.ravel()
    inv = array([[m5*m9-m6*m8, m3*m8-m2*m9, m2*m6-m3*m5],
                    [m6*m7-m4*m9, m1*m9-m3*m7, m3*m4-m1*m6],
                    [m4*m8-m5*m7, m2*m7-m1*m8, m1*m5-m2*m4]])
    return inv / dot(inv[0], m[:, 0])

class PoseEstimator():
    def __init__(self):

        rospy.init_node("Pose_estimator")

        # Transformations
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Coefficients for calculating variance from relative pose
        self.K1 = 7e10
        self.A1 = 3.14
        self.K2 = 9.5e8
        self.A2 = 3.13
        self.K3 = 4e9
        self.A3 = 3.11

        # Coefficients for normalizing surface
        self.norm_length = 0.15
        self.norm_diff = 0.1
        self.p1 = -self.norm_diff/self.norm_length
        self.q1 = (1-self.norm_diff)-pi*self.p1
        self.p2 = self.norm_diff/self.norm_length
        self.q2 = (1-self.norm_diff)-pi*self.p2

        self.valid_marker_flag = False
        self.seq = 0

        cov_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # ROS parameters
        self.markers_map = rospy.get_param("~markers_map", "/home/patrik/catkin_ws/src/mech_ros/map/muzeum_aruco_markers.yaml")
        self.min_covariance = rospy.get_param("~min_covariance", 0.0001)
        self.min_surface = rospy.get_param("~min_surface", 1800)
        self.use_multivariate_gaussian_product = rospy.get_param("~use_multivariate_product", False)
        marker_detector_topic = rospy.get_param("~marker_detector_topic", "/markers")
        estimated_pose_topic = rospy.get_param("~estimated_pose_topic", "/estimated_pose_markers")


        # ROS subscribers and publishers
        rospy.Subscriber(marker_detector_topic, MarkerList, self.marker_cb)
        self.pose_publisher = rospy.Publisher(estimated_pose_topic, PoseWithCovarianceStamped, queue_size=10)
        
        # Load markers map
        self.landmarks = {}
        self.landmarks_frame_id = ""
        self.load_map_file()


        cov_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.estimated = PoseWithCovarianceStamped()
        self.estimated.pose.covariance = cov_pose
        self.estimated.header.frame_id = self.landmarks_frame_id 

    def load_map_file(self):
        with open(self.markers_map, 'r') as stream:
            data_loaded = yaml.load(stream)
            self.landmarks = data_loaded["landmarks"]
            self.landmarks_frame_id = data_loaded["frame_id"]


    def calculateCovariances(self, S_rel, Yaw_rel, Marker_id):
        # Compute covariance matrix of measurement in local coordinate frame
        S_norm = self.B(Yaw_rel)*S_rel/(abs(cos(Yaw_rel)))
        covarianceMeas = zeros([3,3])
        covarianceMeas[0,0] = self.K1/(S_norm**self.A1)
        covarianceMeas[1,1] = self.K2/(S_norm**self.A2)
        covarianceMeas[2,2] = self.K3/(S_norm**self.A3)
        covarianceMeas[covarianceMeas < self.min_covariance] = self.min_covariance

        # Load covariances from markers .yaml file
        v_xx, v_yy, _, _, _, v_FF = self.landmarks[Marker_id]["Covariances"]
        # Assing loaded variances to covariance matrix of marker placement
        covarianceTrans = zeros([3,3])
        covarianceTrans[0,0] = v_xx
        covarianceTrans[1,1] = v_yy
        covarianceTrans[2,2] = v_FF
       
        return covarianceMeas, covarianceTrans

    def B(self, Yaw_rel):
        # Calculate value for compenzation of bigger variance for Yaw_rel~=pi
        Yaw_abs = abs(Yaw_rel%(2*pi))
        if Yaw_abs > (pi - self.norm_length) and Yaw_abs <= pi:
            return self.p1*Yaw_abs + self.q1
        elif Yaw_abs > pi and Yaw_abs < (self.norm_length + pi):
            return self.p2*Yaw_abs + self.q2
        else:
            return 1.0

    def limitMatrix(self, A):
        m_abs = (abs(A) < self.min_covariance)
        m_neg = (A < 0)
        m_pos = (A > 0)
        m_abs_neg = multiply(m_abs, m_neg)
        m_abs_pos = multiply(m_abs, m_pos)
        A[m_abs_neg] = -self.min_covariance
        A[m_abs_pos] = self.min_covariance
        return A

    def calculateTransformedCovariance(self, a_x, a_y, fi, x, y, Fi, S_rel, Marker_id):

        ## Parameters
        # a_x,a_y translation from marker to camera in camera coordinate system
        # fi rotation from marker to camera
        # x,y, translation from global frame to marker
        # Fi rotation from global frame to marker


        # Compute Jacobians
        # J_a - Jacobian of point measurement
        J_a = zeros([3,3])
        J_a[0,0] = cos(fi+Fi)
        J_a[0,1] = -sin(fi+Fi)
        J_a[0,2] = -a_x*sin(fi+Fi) - a_y*cos(fi+Fi)
        J_a[1,0] = sin(fi+Fi)
        J_a[1,1] = cos(fi+Fi)
        J_a[1,2] = a_x*cos(fi+Fi) - a_y*sin(fi+Fi)
        J_a[2,2] = 1

        # J_p - Jacobian of transformation
        J_p = zeros([3,3])
        J_p[0,0] = 1
        J_p[0,2] = -a_x*sin(fi+Fi) - a_y*cos(fi+Fi)
        J_p[1,1] = 1
        J_p[1,2] = a_x*cos(fi+Fi) - a_y*sin(fi+Fi)
        J_p[2,2] = 1

        # Compute transformed covariance matrix of a': Cov_a = J_a.covarianceMeas.J_a' + J_p.covarianceTrans.J_p'
        covarianceMeas, covarianceTrans = self.calculateCovariances(S_rel, fi, Marker_id)
        cov_a = dot(dot(J_a, covarianceMeas), J_a.T) + dot(dot(J_p, covarianceTrans), J_p.T)

        # Make sure no element is smaller than minimal
        cov_a = self.limitMatrix(cov_a)
        return cov_a

    # Compute product for n gaussians
    def multiVariateGaussianProduct(self, Mean_vectors, Cov_matrices):


        matrices_num = Mean_vectors.shape[0]
        variate_num = Mean_vectors.shape[1]

        # Initialize resulting matrices
        Cov_matrix = Cov_matrices[0]
        Mean_vector = Mean_vectors[0]

        # Initialize information matrices
        inf_vec = zeros([variate_num,1])
        inf_mat = zeros([variate_num,variate_num])
        inf_mat_sum = zeros([variate_num,variate_num])


        # Compute information matrix and information vector and then convert to mean vector and covariance matrix
        try:
            for i in range(matrices_num):
                
                inf_mat = inversion(Cov_matrices[i])
                inf_mat_sum += inf_mat
                inf_vec += dot(inf_mat, Mean_vectors[i])


            Cov_matrix = inversion(inf_mat_sum)
            Mean_vector = dot(Cov_matrix, inf_vec)

            return Mean_vector, Cov_matrix

        except:
            if 'Singular matrix' in str(err):

                try: 
                    # Initialize matrices
                    Cov_matrix = Cov_matrices[0]
                    Mean_vector = Mean_vectors[0]

                    for i in range(matrices_num-1):

                        # Using Choleskeho decomposition
                        cholesky_matrix = linalg.cholesky(Cov_matrix + Cov_matrices[i+1])
                        chol_cov1 = linalg.solve(cholesky_matrix, Cov_matrix)
                        chol_cov2 = linalg.solve(cholesky_matrix, Cov_matrices[i+1])
                        chol_mean1 = linalg.solve(cholesky_matrix, Mean_vector)
                        chol_mean2 = linalg.solve(cholesky_matrix, Mean_vectors[i+1])

                        Cov_matrix = dot(chol_cov1.T,chol_cov2)
                        Mean_vector = dot(chol_cov2.T,chol_mean1) + dot(chol_cov1.T,chol_mean2)

                    return Mean_vector, Cov_matrix             

                except linalg.LinAlgError as err:
                    if 'positive definite' in str(err):

                        try:
                            # Initialize matrices
                            Cov_matrix = Cov_matrices[0]
                            Mean_vector = Mean_vectors[0]

                            for i in range(matrices_num-1):

                                cholesky_matrix = lin.cholesky((Cov_matrix + Cov_matrices[i+1]), lower=True,check_finite=False)
                                chol_cov1 = lin.solve_triangular(cholesky_matrix, Cov_matrix,lower=True,check_finite=False)
                                chol_cov2 = lin.solve_triangular(cholesky_matrix, Cov_matrices[i+1],lower=True,check_finite=False)
                                chol_mean1 = lin.solve_triangular(cholesky_matrix, Mean_vector,lower=True,check_finite=False)
                                chol_mean2 = lin.solve_triangular(cholesky_matrix, Mean_vectors[i+1],lower=True,check_finite=False)
            
                                Cov_matrix = dot(chol_cov1.T,chol_cov2)
                                Mean_vector = dot(chol_cov2.T,chol_mean1) + dot(chol_cov1.T,chol_mean2)

                            return Mean_vector, Cov_matrix

                        except:
                            return Mean_vectors[0], Cov_matrices[0]

                    else:
                        rospy.logerr("Unexpected error when computing multivariate product")

            else:
                rospy.logerr("Unexpected error when computing multivariate product")
                return Mean_vectors[0], Cov_matrices[0]


    ######## Predelat na estimaci polohy teziste vzhledem ke glob. sour. systemu, momentalne je to vzhledem ke kamere
    # translace z base_link to camera Translation: [0.088, 0.000, 0.020]
    # pripoctu k x_m a z_m (ZAPORNE HODNOTY)


    def averageMeasurement(self, Mean_vectors, Cov_matrices):
        sum_sigma_x = sum_n(1/Cov_matrices[:,0,0])
        sum_sigma_y = sum_n(1/Cov_matrices[:,1,1])
        sum_sigma_fi = sum_n(1/Cov_matrices[:,2,2])


        matrices_num = Mean_vectors.shape[0]
        mean_vec = zeros([3,1])

        avg_cov = sum_n(Cov_matrices, axis = 0)/(matrices_num**2)

        for i in range(matrices_num):
            mean_vec[0,0] += Mean_vectors[i,0,0]/(sum_sigma_x*Cov_matrices[i,0,0])
            mean_vec[1,0] += Mean_vectors[i,1,0]/(sum_sigma_y*Cov_matrices[i,1,1])
            mean_vec[2,0] += Mean_vectors[i,2,0]/(sum_sigma_fi*Cov_matrices[i,2,2])

        return mean_vec, avg_cov

    def marker_cb(self, Markers):

        self.valid_marker_flag = False
        time_to_compare = Markers.header.stamp
        mark_num = len(Markers.markers)
        mean = zeros((mark_num,3,1))
        covariances = zeros((mark_num,3,3))
        i = 0

        for marker in Markers.markers:
            Marker_id = int(marker.id)

            surface = marker.surface
            if surface < self.min_surface:

                # Reshape allocated arrays
                mean = mean[:-1]
                covariances = covariances[:-1]
                continue

            time = marker.header.stamp          
            if time_to_compare != time:

                # Reshape allocated arrays
                mean = mean[:-1]
                covariances = covariances[:-1]
                continue
            
            # Check if marker is in dictionary
            if not(any(Marker_id == key for key in self.landmarks)):
                # Reshape allocated arrays
                mean = mean[:-1]
                covariances = covariances[:-1]
                continue

            # Publish transformations from camera to markers for visualization
            t = TransformStamped()
            t.child_frame_id = "marker_" + marker.id
            t.header.frame_id = Markers.header.frame_id
            t.header.stamp = time
            # Have to be inverted translation - because of the way Aruco_detect.py transform initial vector
            t.transform.translation.x = -marker.pose.position.x
            t.transform.translation.y = -marker.pose.position.y
            t.transform.translation.z = marker.pose.position.z

            q_convert = quaternion_from_euler(-marker.pose.orientation.r,-marker.pose.orientation.p, -marker.pose.orientation.y)
            quat = Quaternion(*q_convert)
            t.transform.rotation = quat
            self.tfBroadcaster.sendTransform(t)

            rospy.logdebug("Marker # %s spotted",Marker_id)

            self.valid_marker_flag = True
            x_m = marker.pose.position.x - 0.088
            y_m = marker.pose.position.y
            # z_m = marker.pose.position.z - 0.02
            # R_m = marker.pose.orientation.r
            # P_m = marker.pose.orientation.p
            Y_m = marker.pose.orientation.y


            # Assign markers pose
            x_mm,y_mm, _ = self.landmarks[Marker_id]["Translation"]
            _, _, Y_mm =self.landmarks[Marker_id]["RPY"]
        

            mean[i,0,0] = (x_mm + cos(Y_mm+Y_m)*x_m - sin(Y_mm+Y_m)*y_m)
            mean[i,1,0] = (y_mm + sin(Y_mm+Y_m)*x_m + cos(Y_mm+Y_m)*y_m)
            
            # Apply mod to Yaw - without mod average Yaw value jump when combining
            if Y_m < 0:
                Y_m += 2*pi

            if Y_mm < 0:
                Y_mm += 2*pi

            mean[i,2,0] = Y_mm + Y_m


            # Compute covariance matrix and transformed covariance matrix and store them
            covariances[i] = self.calculateTransformedCovariance(mean[i,0,0],mean[i,1,0],Y_m,x_mm,y_mm,Y_mm, surface, Marker_id)
            i =+1


        if self.valid_marker_flag:
            self.seq += 1
            # Compute estimated mean vector and covariance matrix from all measurement
            if mean.shape[0] > 1:
                if self.use_multivariate_gaussian_product:
                    mean_tr, cov_tr = self.multiVariateGaussianProduct(mean, covariances)
                else:
                    mean_tr, cov_tr = self.averageMeasurement(mean, covariances)
            else:
                mean_tr, cov_tr = mean[0], covariances[0]

            cov_tr = self.limitMatrix(cov_tr)
           
            # Fill estimated pose
            self.estimated.header.stamp = time
            self.estimated.header.seq = self.seq
            self.estimated.pose.pose.position.x= mean_tr[0]
            self.estimated.pose.pose.position.y= mean_tr[1]
            q_convert = quaternion_from_euler(0,0,mean_tr[2])
            self.estimated.pose.pose.orientation = Quaternion(*q_convert)


            # Fill covariance

            self.estimated.pose.covariance[0] = cov_tr[0,0]
            self.estimated.pose.covariance[1] = cov_tr[0,1]
            self.estimated.pose.covariance[5] = cov_tr[0][2]
            self.estimated.pose.covariance[6] = self.estimated.pose.covariance[1]
            self.estimated.pose.covariance[7] = cov_tr[1,1]
            self.estimated.pose.covariance[11] = cov_tr[1][2]
            self.estimated.pose.covariance[30] = self.estimated.pose.covariance[5]
            self.estimated.pose.covariance[31] = self.estimated.pose.covariance[11]
            self.estimated.pose.covariance[35] = cov_tr[2,2]

            self.pose_publisher.publish(self.estimated)



if __name__ == '__main__':
    try:
        poseEstimator = PoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
