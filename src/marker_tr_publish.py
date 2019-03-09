#!/usr/bin/env python

import rospy
import sys
import yaml

# import tf2_ros

from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


from perception_msgs.msg import MarkerList
from perception_msgs.msg import Marker


# Define topics
marker_detector_topic = "/markers"
publish_estimated_pose = "/estimated_pose_markers"
frame_id = "map"

pi = math.pi
seq = 0

# Marker_map
# map_filename = "/home/patrik/catkin_ws/src/mech_ros/map/muzeum_aruco_markers.yaml"
map_filename = "/home/patrik/catkin_ws/src/mech_ros/map/marker_kalibrace2.yaml"


# Read marker map
with open(map_filename, 'r') as stream:
    data_loaded = yaml.load(stream)
    landmarks = data_loaded["landmarks"]
    landmarks_frame_id = data_loaded["frame_id"]


# Cov matrix without correction
# cov_pose = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.05]

cov_pose = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

range_multiplier = 2
theta_multiplier = 6

######## Predelat na estimaci polohy teziste vzhledem ke glob. sour. systemu, momentalne je to vzhledem ke kamere
# translace z base_link to camera Translation: [0.088, 0.000, 0.020]
# pripoctu k x_m a z_m


def marker_callback(Markers):
    global seq
    x = []
    y = []
    z = []
    yaw = []
    range_m = []
    theta_m = []
    flag = False
    time_to_compare = Markers.header.stamp

    for marker in Markers.markers:
        id = int(marker.id)
        time = marker.header.stamp

        if time_to_compare != time:
            continue

        flag = True
        x_m = marker.pose.pose.position.x + 0.088
        # x_m = marker.pose.pose.position.xa
        y_m = marker.pose.pose.position.y
        z_m = marker.pose.pose.position.z + 0.02
        q_x = marker.pose.pose.orientation.x
        q_y = marker.pose.pose.orientation.y
        q_z = marker.pose.pose.orientation.z
        q_w = marker.pose.pose.orientation.w
        R_m,P_m,Y_m = euler_from_quaternion([q_x,q_y,q_z,q_w])

        # otoceni  - beru rotaci vzhledem k markeru, pricteni - jiny souradny system - potom predelat 
        Y_m = -(-pi/2 +Y_m)

        # Vypocet  vzdalenosti a natoceni od markeru pro korekci kovariancni matice
        range_m.append((x_m**2 + y_m**2)**0.5)
        theta_m.append(abs(Y_m-pi/2))

        # otoceni kvuli markeru
        x_marker = (math.cos(Y_m)*x_m + math.sin(Y_m)*y_m)
        y_marker = (-math.sin(Y_m)*x_m + math.cos(Y_m)*y_m)
        z_marker = -z_m

        x_mm,y_mm,z_mm = landmarks[id]["Translation"]
        R_mm, P_mm, Y_mm =landmarks[id]["RPY"]

        # u Y_mm je - , protoze otacim z markeru do glob. sour.sys.
        x.append(x_mm + math.cos(-Y_mm)*x_marker + math.sin(-Y_mm)*y_marker)
        y.append(y_mm - math.sin(-Y_mm)*x_marker + math.cos(-Y_mm)*y_marker)
        z.append(z_mm + z_marker)
        # pricteni pi - otoceni, yaw se musi odecist
        yaw.append((Y_mm - Y_m + pi))


    if flag:
        seq +=1
        estimated = PoseWithCovarianceStamped()
        estimated.header.frame_id = frame_id
        estimated.header.stamp = time
        estimated.header.seq = seq
        estimated.pose.pose.position.x= sum(x)/len(x)
        estimated.pose.pose.position.y= sum(y)/len(y)
        estimated.pose.pose.position.z= sum(z)/len(z)
        q_convert = quaternion_from_euler(0,0,sum(yaw)/len(yaw))
        estimated.pose.pose.orientation = Quaternion(*q_convert)

        # Average range and angle for covariance
        range_av = sum(range_m)/len(range_m)
        theta_av = sum(theta_m)/len(theta_m)
        cov_corrected = [(range_av+theta_av)*x for x in cov_pose]
        estimated.pose.covariance = cov_corrected
        pose_publisher.publish(estimated)




if __name__ == '__main__':
    rospy.init_node('marker_pose', anonymous=True)
    marker_subscriber = rospy.Subscriber(marker_detector_topic, MarkerList, marker_callback)
    pose_publisher = rospy.Publisher(publish_estimated_pose, PoseWithCovarianceStamped,queue_size=10)
    rospy.spin()