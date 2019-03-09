#!/usr/bin/env python



import numpy as np
import cv2
import yaml
# import time
import rospy
import math



from geometry_msgs.msg import PoseWithCovarianceStamped
from marker_msgs.msg import MarkerDetection
from marker_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Generate dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

# Define topics
marker_detector_topic = "/markers"
image_topic = "/pi_camera/image_raw"
frame_id = "front_camera_link"

# Define calibration filename
# calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml"
calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera_Gazebo.yaml"



## Define Aruco Params
arucoParams = cv2.aruco.DetectorParameters_create()
arucoParams.minMarkerPerimeterRate = 0.08
arucoParams.minCornerDistanceRate = 0.06
arucoParams.cornerRefinementMethod = 1
arucoParams.cornerRefinementMaxIterations = 35
arucoParams.markerBorderBits = 1
arucoParams.maxErroneousBitsInBorderRate = 0.4

# camera_matrix=np.array([[1337.3442041956487, 0.0, 829.6600891797842],[0.0, 1337.5855702824913, 485.17756483016257],[0.0,0.0,1.0]])
# dist_coeff = np.array([0.1954298919955456, -0.16441936311127164, -0.004914964353264576, -0.00014855354072454622,-1.0316591472028718])



## Initialization
bridge = CvBridge()
markerIds = np.array([])
markerCorners = np.array([])
rejectedImgPoints = np.array([])
markerLength = 0.1778*0.75



# Vytvoreni rotacni matice odpovidajici RPY [pi/2,0,pi/2]
# rot = np.array([[0, -1, 0],
#                 [0, 0, -1],
#                 [1, 0, 0]])

# # Matrix for conversion from ROS frame to OpenCV for camera
# rot_cam = np.array([[  0.0,  0.0,   1.0],
#  [  -1.0,   0.0,  0.0],
#  [  0.0,   -1.0,   0.0]])

#  # Matrix for conversion from ROS frame to OpenCV for marker
# rot_mark = np.array([[  0.0,  0.0,   1.0],
#  [  1.0,   0.0,  0.0],
#  [  0.0,   1.0,   0.0]])

# Matrix for conversion from ROS frame to OpenCV in camera
R_ROS_O_camera = np.array([[  0.0,  0.0,   1.0],
 [  -1.0,   0.0,  0.0],
 [  0.0,   -1.0,   0.0]])

 # Matrix for conversion from OpenCV frame to ROS in marker
R_O_ROS_marker = np.array([[  0.0,  1.0,   0.0],
 [  0.0,   0.0,  1.0],
 [  1.0,   0.0,   0.0]])


## Load coefficients
def load_coefficient(calibration_file):
    with open(calibration_file) as stream:
        try:
            data_loaded = yaml.load(stream)
            c_matrix = np.array(data_loaded["camera_matrix"]["data"]).reshape((3,3))
            dist_coeff = np.array(data_loaded["distortion_coefficients"]["data"])
            return c_matrix, dist_coeff
        except yaml.YAMLError as exc:
            print(exc)
    # with np.load('/home/patrik/catkin_ws/src/mech_ros/map/camCal2.npz') as X:
    #     c_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    # return c_matrix, dist_coeff


_FLOAT_EPS_4 = np.finfo(float).eps * 4.0

def rotationMatrixToEulerAngles(M, cy_thresh=None):
    # cy_thresh : None or scalar, optional
    #    threshold below which to give up on straightforward arctan for
    #    estimating x rotation.  If None (default), estimate from
    #    precision of input. Source : http://www.graphicsgems.org/

    if cy_thresh is None:
        try:
            cy_thresh = np.finfo(M.dtype).eps * 4
        except ValueError:
            cy_thresh = _FLOAT_EPS_4
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
    # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
    cy = math.sqrt(r33*r33 + r23*r23)
    if cy > cy_thresh: # cos(y) not close to zero, standard form
        z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
        # so r21 -> sin(z), r22 -> cos(z) and
        z = math.atan2(r21,  r22)
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = 0.0
    return [x, y, z]

## Tohle je jine poradi rotaci (x,y,z)
# def rotationMatrixToEulerAngles(R):
#     # https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
#     roll = math.atan2(R[1,2], R[2,2])
#     pitch = math.atan2(-R[0,2], ((R[0,0]**2+R[0,1]**2)**0.5))
#     s1 = math.sin(roll)
#     c1 = math.cos(roll)
#     yaw = math.atan2((s1*R[2,0]-c1*R[1,0]), (c1*R[1,1] - s1*R[2,1]))

#     return [roll, pitch, yaw]


## Tohle je spravne, ale neni tu resena singularita
# def rotationMatrixToEulerAngles(R) :
#      # TO DO - avoid singularity

#     # pitch = math.asin(R[2,0])
#     # yaw = math.atan2(R[1,0], R[0,0])
#     # roll = math.atan2(R[2,1], R[2,2])

#     if R[0,2] > 1:
#         R[0,2] = 1
#     elif R[0,2]< -1:
#         R[0,2] = -1
#     pitch = math.asin(R[0,2])
#     yaw = math.atan2(-R[0,1], R[0,0])
#     roll = math.atan2(-R[1,2], R[2,2])

#     return [roll, pitch, yaw]


def rotationToQuaternion(M):
    trace = M[0,0] + M[1,1] + M[2,2]
    q = [0,0,0,0]
    if trace>0:
        s = math.sqrt(trace+1.0)    
        q[3] = s*0.5
        s = 0.5/s
        q[0] = (M[2,1]-M[1,2])*s
        q[1] = (M[0,2]-M[2,0])*s
        q[2] = (M[1,0]-M[0,1])*s

    else:
        if M[0,0] < M[1,1]:
            if M[1,1] < M[2,2]:
                i = 2
            else:
                i = 1
        else:
            if M[0,0] < M[2,2]:
                i = 2
            else:
                i = 0
        j = (i+1)%3
        k = (i+1)%3
        s = math.sqrt(M[i,i] - M[j,j] - M[k,k] +1.0)
        q[i] = s*0.5
        s = 0.5/s
        q[3] = (M[k,j] - M[j,k])*s
        q[j] = (M[j,i] - M[i,j])*s
        q[k] = (M[k,i] - M[i,k])*s
        norm = (q[0]**2 + q[1]**2+q[2]**2 + q[3]**2)**0.5
        q = q/norm

    return q

def image_callback(frame):
    global bridge
    try:
        image = bridge.imgmsg_to_cv2(frame, desired_encoding="mono8")
        # image = bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    time = rospy.Time.now()
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    markerCorners,markerIds,rejectedImgPoints = cv2.aruco.detectMarkers(image,dictionary,parameters = arucoParams, cameraMatrix = camera_matrix,
    distCoeff = dist_coeff )


    if len(markerCorners) > 0:
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerLength, camera_matrix, dist_coeff) # For a single marker
        MarkerList = MarkerDetection()
        MarkerList.header.stamp = time
        MarkerList.header.frame_id = frame_id
        MarkerList.distance_max = 6
        MarkerList.distance_max_id = 6
        MarkerList.distance_min = 0.1
        # MarkerList.fov_horizontal = 1.085594795
        MarkerList.fov_horizontal = 6
        # MarkerList.fov_vertical = 0.872664626
        MarkerList.fov_vertical = 6        
        MarkerList.view_direction.x = 0
        MarkerList.view_direction.y = 0
        MarkerList.view_direction.z = 0
        MarkerList.view_direction.w = 1
        MarkerList.type = "artag"


        for i in range(markerIds.size):
            
            # Fill MarkerList with each marker
            aruco_Marker = Marker()
            aruco_Marker.ids = [int(markerIds[i,0])]
            aruco_Marker.ids_confidence = [1]


            # Prevedeni rodrigues vectoru na rotacni matici
            Rmat = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec[i,0],Rmat)

            # Convert from Opencv frame to ROS frame in camera
            # R = np.dot(R_ROS_O_camera, Rmat)

            # Convert inverted matrix from Opencv frame to ROS frame in marker
            # R = np.dot(R, R_O_ROS_marker)
            # quat = rotationToQuaternion(R)
            quat = rotationToQuaternion(Rmat)

            # Fill Marker orientation vector
            aruco_Marker.pose.orientation.x = quat[0]
            aruco_Marker.pose.orientation.y = quat[1]
            aruco_Marker.pose.orientation.z = quat[2]
            aruco_Marker.pose.orientation.w = quat[3]


            # Coordinate vector of camera position from marker in camera coordinate frame
            # aruco_Marker.pose.position.x = tvec[i,0,2]
            # aruco_Marker.pose.position.y = tvec[i,0,0]
            # aruco_Marker.pose.position.z = tvec[i,0,1]
            aruco_Marker.pose.position.x = tvec[i,0,0]
            aruco_Marker.pose.position.y = tvec[i,0,1]
            aruco_Marker.pose.position.z = tvec[i,0,2]

            # All coordinates are in marker frame
            MarkerList.markers.append(aruco_Marker)

        marker_publisher.publish(MarkerList)

        # cv2.imshow('frame',image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
            





if __name__ == '__main__':
    rospy.init_node('aruco_detect', anonymous=True)
    camera_matrix, dist_coeff = load_coefficient(calibration_file)
    image_subscriber = rospy.Subscriber(image_topic, Image, image_callback )
    marker_publisher = rospy.Publisher(marker_detector_topic, MarkerDetection,queue_size=10)
    rospy.spin()

