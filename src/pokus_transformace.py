#!/usr/bin/env python

import rospy
import sys
import yaml

# import tf2_ros

from geometry_msgs.msg import Quaternion, Transform
# from geometry_msgs.msg import Vector3
# from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


from perception_msgs.msg import MarkerList
from perception_msgs.msg import Marker

# Define topics
marker_detector_topic = "/markers"
publish_estimated_pose = "/estimated_pose_markers"
frame_id = "/world"

seq = 0
pi = math.pi
# Read marker map
with open("/home/patrik/catkin_ws/src/mech_ros/map/muzeum_aruco_markers.yaml", 'r') as stream:
    data_loaded = yaml.load(stream)
    landmarks = data_loaded["landmarks"]
    landmarks_frame_id = data_loaded["frame_id"]

cov_pose = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]


# global seq
x = []
y = []
z = []
yaw = []
range_m = []
theta_m = []
# time = rospy.Time.now()

marker1 = Marker()
marker1.id = 0
marker1.header.seq = 0
marker1.pose.pose.position.x = 2
marker1.pose.pose.position.y = 0.3
marker1.pose.pose.position.z =  0.324831
quater1 = quaternion_from_euler(0.0, 0.0, pi/2)
marker1.pose.pose.orientation = quater1

marker2 = Marker()
marker2.id = 1
marker2.header.seq = 0
marker2.pose.pose.position.x = 3
marker2.pose.pose.position.y = -3.062350
marker2.pose.pose.position.z =  0.554149
quater2 = quaternion_from_euler(0.0, 0.0, pi/2)
marker2.pose.pose.orientation = quater2

Markers = MarkerList()
Markers.markers = [marker1,marker2]

# print(marker2.pose.pose.position)
# print(marker2.pose.pose.position.x)

for marker in Markers.markers:
    id = marker.id
    x_m = marker.pose.pose.position.x
    y_m = marker.pose.pose.position.y
    z_m = marker.pose.pose.position.z
    R_m,P_m,Y_m = euler_from_quaternion(marker.pose.pose.orientation)
    
    # otoceni  - beru rotaci vzhledem k markeru, pricteni - jiny souradny system - potom predelat 
    Y_m = math.pi/2 +Y_m

    # Vypocet  vzdalenosti a natoceni od markeru pro korekci kovariancni matice
    range_m.append((x_m**2 + y_m**2)**0.5)
    theta_m.append(abs(math.atan(y_m/x_m)))

    # rot_matrix = np.matrix([[np.cos(Y_m),np.sin(Y_m)],[-np.sin(Y_m),np.cos(Y_m)]])
    # trans_matrix = np.matrix([[x_m],[y_m]])
    # otoceni kvuli markeru
    x_marker = -(math.cos(Y_m)*x_m + math.sin(Y_m)*y_m)
    y_marker = -(math.sin(Y_m)*x_m + math.cos(Y_m)*y_m)
    z_marker = -z_m

    x_mm,y_mm,z_mm = landmarks[id]["Translation"]
    R_mm, P_mm, Y_mm =landmarks[id]["RPY"]

    x.append(x_mm + math.cos(Y_mm)*x_marker + math.sin(Y_mm)*y_marker)
    y.append(y_mm - math.sin(Y_mm)*x_marker + math.cos(Y_mm)*y_marker)
    z.append(z_mm + z_marker)
    # pricteni pi - otoceni, yaw se musi secit
    yaw.append((Y_mm + Y_m))


seq +=1
estimated = PoseWithCovarianceStamped()
estimated.header.frame_id = frame_id
# estimated.pose.header = time
estimated.header.seq = seq
estimated.pose.pose.position.x= sum(x)/len(x)
estimated.pose.pose.position.y= sum(y)/len(y)
estimated.pose.pose.position.z= sum(z)/len(z)
estimated.pose.pose.orientation = quaternion_from_euler(0,0,sum(yaw)/len(yaw))
# Average range and angle for covariance
range_av = sum(range_m)/len(range_m)
theta_av = sum(theta_m)/len(theta_m)
cov_corrected = [(range_av+theta_av)*x for x in cov_pose]
estimated.pose.covariance = cov_corrected

# pose_publisher.publish(estimated)

print(x)
print(y)
print(yaw)
print(estimated.pose)
print(estimated.pose.covariance)

