#!/usr/bin/env python

import rospy
import sys
import yaml

# import tf2_ros

from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import pi, cos, sin, atan2
from numpy import zeros, dot, linalg
from mech_ros_msgs.msg import MarkerList
from mech_ros_msgs.msg import Marker


# Define topics
marker_detector_topic = "/markers"
publish_estimated_pose = "/estimated_pose_markers"
frame_id = "map"


seq = 0

# Marker_map
map_filename = "/home/patrik/catkin_ws/src/mech_ros/map/muzeum_aruco_markers.yaml"
# map_filename = "/home/patrik/catkin_ws/src/mech_ros/map/marker_kalibrace.yaml"

# Coefficients for calculating variance from relative pose
K1 = 1.5e10
A1 = 3.3
K2 = 50
A2 = 2.0
K3 = 20
A3 = 1.1

# Covariances for transform
# TO DO
min_covariance = 1e-4

# Read marker map
with open(map_filename, 'r') as stream:
    data_loaded = yaml.load(stream)
    landmarks = data_loaded["landmarks"]
    landmarks_frame_id = data_loaded["frame_id"]



def calculateCovariances(S_rel, Yaw_rel):
    S_norm = S_rel/max(abs(cos(Yaw_rel)),abs(sin(Yaw_rel)))
    covarianceMeas = zeros([2,2])
    covarianceMeas[0,0] = K1/(S_norm**A1)
    covarianceMeas[1,1] = K2/(S_norm**A2)
    covarianceMeas[covarianceMeas < min_covariance] = min_covariance

    # Constant covariance for a_x and a_y, Fi -  marker in global coordinate system
    # distance : 0.01**2, angle~: 0.0175**2 
    covarianceTrans = zeros([3,3])
    covarianceTrans[0,0] = 0.001
    covarianceTrans[1,1] = 0.001
    # covariance in angle = cov_fi + cov_Fi
    covarianceTrans[2,2] = K3/(S_norm**A3) + 0.0005
    if covarianceTrans[2,2] < min_covariance:
        covarianceTrans[2,2] = min_covariance
    
    return covarianceMeas, covarianceTrans


def calculateTransformedCovariance(a_x, a_y, fi, x, y, Fi, covarianceMeas, covarianceTrans):
    ############ TO DO ###############x
    
    ## Parameters
    # a_x,a_y translation from marker to camera in camera coordinate system
    # fi rotation from marker to camera
    # x,y, translation from global frame to marker
    # Fi rotation from global frame to camera

    # Compute Jacobians
    # J_a - Jacobian of point measurement
    J_a = zeros([2,2])
    J_a[0,0] = cos(fi+Fi)
    J_a[0,1] = -sin(fi+Fi)
    J_a[1,0] = sin(fi+Fi)
    J_a[1,1] = cos(fi+Fi)

    # J_p - Jacobian of transformation
    J_p = zeros([2,3])
    J_p[0,0] = 1
    J_p[0,2] = -a_x*sin(fi+Fi) - a_y*cos(fi+Fi)
    J_p[1,1] = 1
    J_p[1,2] = a_x*cos(fi+Fi) - a_y*sin(fi+Fi)

    # Compute transformed covariance matrix of a'- Cov_a = J_a.covarianceMeas.J_a' + J_p.covarianceTrans.J_p'
    cov_a = dot(dot(J_a, covarianceMeas), J_a.T) + dot(dot(J_p, covarianceTrans), J_p.T)

    # Make 3x3 covariance matrix for transformed - a_x', a_y', theta  (theta = Fi + fi)
    transformedCovariance = zeros([3,3])
    transformedCovariance[:2,:2] = cov_a
    transformedCovariance[2,2] = covarianceTrans[2][2]

    return transformedCovariance

# Compute product for n gaussians
def multiVariateGaussianProduct(Mean_vectors, Cov_matrices):


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

            inf_mat = linalg.inv(Cov_matrices[i])
            inf_mat_sum += inf_mat
            inf_vec += dot(inf_mat, Mean_vectors[i])


        Cov_matrix = linalg.inv(inf_mat_sum)
        Mean_vector = dot(Cov_matrix, inf_vec)

        return Mean_vector, Cov_matrix

    except linalg.LinAlgError as err:
        if 'Singular matrix' in str(err):

            # Initialize matrices
            Cov_matrix = Cov_matrices[0]
            Mean_vector = Mean_vectors[0]

            for i in range(matrices_num-1):

                # Using Choleskeho decomposition
                try:
                    cholesky_matrix = linalg.cholesky(Cov_matrix + Cov_matrices[i+1])
                except:
                    print(Cov_matrix + Cov_matrices[i+1])
                    rospy.loginfo("Unexpected error when computing cholesky decomposition")
                chol_cov1 = linalg.solve(cholesky_matrix, Cov_matrix)
                chol_cov2 = linalg.solve(cholesky_matrix, Cov_matrices[i+1])
                chol_mean1 = linalg.solve(cholesky_matrix, Mean_vector)
                chol_mean2 = linalg.solve(cholesky_matrix, Mean_vectors[i+1])

                Cov_matrix = dot(chol_cov1.T,chol_cov2)
                Mean_vector = dot(chol_cov2.T,chol_mean1) + dot(chol_cov1.T,chol_mean2)

            return Mean_vector, Cov_matrix             

            
        else:
            rospy.logerr("Unexpected error when computing multivariate product")


######## Predelat na estimaci polohy teziste vzhledem ke glob. sour. systemu, momentalne je to vzhledem ke kamere
# translace z base_link to camera Translation: [0.088, 0.000, 0.020]
# pripoctu k x_m a z_m (ZAPORNE HODNOTY)


def marker_callback(Markers):
    global seq

    flag = False
    time_to_compare = Markers.header.stamp
    mark_num = len(Markers.markers)
    mean = zeros((mark_num,3,1))
    covariances = zeros((mark_num,3,3))
    i = 0


    for marker in Markers.markers:
        id = int(marker.id)

        # Check if marker is in dictionary
        if not(any(id == key for key in landmarks)):
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
        
        surface = marker.surface
        if surface < 1300:

            # Reshape allocated arrays
            mean = mean[:-1]
            covariances = covariances[:-1]
            continue

        flag = True
        x_m = marker.pose.position.x - 0.088
        y_m = marker.pose.position.y
        # z_m = marker.pose.position.z - 0.02
        # R_m = marker.pose.orientation.r
        # P_m = marker.pose.orientation.p
        Y_m = marker.pose.orientation.y
        surface = marker.surface

        # Assign markers pose
        x_mm,y_mm,z_mm = landmarks[id]["Translation"]
        R_mm, P_mm, Y_mm =landmarks[id]["RPY"]
     

        mean[i][0][0] = (x_mm + cos(Y_mm+Y_m)*x_m - sin(Y_mm+Y_m)*y_m)
        mean[i][1][0] = (y_mm + sin(Y_mm+Y_m)*x_m + cos(Y_mm+Y_m)*y_m)
        
        # Apply mod to Yaw - without mod average Yaw value jump when combining
        if Y_m < 0:
            Y_m += 2*pi

        if Y_mm < 0:
            Y_mm += 2*pi

        mean[i][2][0] = Y_mm + Y_m


        # Compute covariance matrix and transformed covariance matrix and store them
        covarianceMeas, covarianceTrans = calculateCovariances(surface ,Y_m)
        covariances[i] = calculateTransformedCovariance(mean[i][0][0],mean[i][1][0],Y_m,x_mm,y_mm,Y_mm, covarianceMeas, covarianceTrans)
        i =+1


    if flag:
        seq +=1
        # If no suitable markers were found
        if mean.size == 0:
            return

        # Compute estimated mean vector and covariance matrix from all measurement
        if mean.shape[0] > 1:
            mean_tr, cov_tr = multiVariateGaussianProduct(mean, covariances)
        
        else:
            mean_tr, cov_tr = mean[0], covariances[0]

        cov_tr[cov_tr < min_covariance] = min_covariance
        print(mean_tr)
          

        # Fill estimated pose
        estimated = PoseWithCovarianceStamped()
        estimated.header.frame_id = frame_id
        estimated.header.stamp = time
        estimated.header.seq = seq
        estimated.pose.pose.position.x= mean_tr[0]
        estimated.pose.pose.position.y= mean_tr[1]
        q_convert = quaternion_from_euler(0,0,mean_tr[2])
        estimated.pose.pose.orientation = Quaternion(*q_convert)


        # Fill covariance
        cov_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cov_pose[0] = cov_tr[0][0]
        cov_pose[1] = cov_tr[0][1]
        # cov_pose[5] = cov_tr[0][2]
        cov_pose[6] = cov_pose[1]
        cov_pose[7] = cov_tr[1][1]
        # cov_pose[11] = cov_tr[1][2]
        # cov_pose[30] = cov_pose[5]
        # cov_pose[31] = cov_pose[11]
        cov_pose[35] = cov_tr[2][2]

        # Scale factor due to multivariate gaussian product is not gaussian without scale

        estimated.pose.covariance = cov_pose
        pose_publisher.publish(estimated)



if __name__ == '__main__':
    rospy.init_node('marker_pose', anonymous=True)
    marker_subscriber = rospy.Subscriber(marker_detector_topic, MarkerList, marker_callback)
    pose_publisher = rospy.Publisher(publish_estimated_pose, PoseWithCovarianceStamped,queue_size=10)
    rospy.spin()