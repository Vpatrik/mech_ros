import numpy as np
import cv2
import yaml
# import time
import rospy
import math

from geometry_msgs.msg import Quaternion, Transform
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from perception_msgs.msg import MarkerList
from perception_msgs.msg import Marker

## Generate dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Define topics
marker_detector_topic = "/markers"
frame_id = "/camera"

# Vytvoreni rotacni matice odpovidajici RPY [pi/2,0,pi/2]
rot = np.array([[0, -1, 0],
                [0, 0, -1],
                [0, 0, 0]])


## Load coefficients
#with open("calibration.yml") as stream:
#	try:
#		data_loaded = yaml.load(stream)
##		c_matrix = data_loaded("camera_matrix")
##		dist_coeff = data_loaded("distortion_coefficients")
#	except yaml.YAMLError as exc:
#        	print(exc)
#fs.open()
##c_matrix = fs.getNode("camera_matrix")
##dist_coeff = fs.getNode("distortion_coefficients")
##print(c_matric.mat)
##print(dist_coeff.real())

## Define Aruco Params
arucoParams = cv2.aruco.DetectorParameters_create()
arucoParams.minMarkerPerimeterRate = 0.02
arucoParams.minCornerDistanceRate = 0.1
#arucoParams.cornerRefinementMethod = "CORNER_REFINE_NONE"
arucoParams.markerBorderBits = 1
arucoParams.maxErroneousBitsInBorderRate = 0.4

camera_matrix=np.array([[1337.3442041956487, 0.0, 829.6600891797842],[0.0, 1337.5855702824913, 485.17756483016257],[0.0,0.0,1.0]])
dist_coeff = np.array([0.1954298919955456, -0.16441936311127164, -0.004914964353264576, -0.00014855354072454622,-1.0316591472028718])



def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
#    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img




## Initialization
markerIds = np.array([])
markerCorners = np.array([])
rejectedImgPoints = np.array([])
markerLength = 0.088
axis = np.float32([[markerLength/2,markerLength/2,0], [-markerLength/2,markerLength/2,0], [-markerLength/2,-markerLength/2,0], [markerLength/2,-markerLength/2,0],
                   [markerLength/2,markerLength/2,markerLength],[-markerLength/2,markerLength/2,markerLength],[-markerLength/2,-markerLength/2,markerLength],[markerLength/2,-markerLength/2,markerLength] ])


# i = 0
# time_last = time.time()

############### Capture stream ###############

######## UDP ####################
#pipe = "udpsrc port=5777 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"

markerIds = [5,4]
tvec = [[1,2,3],[4,5,6]]
rvec = np.array([[1,2,3],[4,5,6]],np.float32)


aruco_MarkerList = MarkerList()

aruco_MarkerList.header.frame_id = frame_id

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
        
    return q

# print(len(markerIds))
for i in range(len(markerIds)):


    # Fill MarkerList with each marker
    aruco_Marker = Marker()
    aruco_Marker.id = markerIds[i]
    aruco_Marker.pose.pose.position.x = float(tvec[i][0])
    aruco_Marker.pose.pose.position.y = float(tvec[i][1])
    aruco_Marker.pose.pose.position.z = float(tvec[i][2])

    Rmat = np.zeros(shape=(3,3))

    cv2.Rodrigues(rvec[i],Rmat)

    # Rotace - prevedeni z framu obrazu opencv do framu ROS
    R = np.dot(rot, Rmat)
    # Prevedni rotacni matice na quaternion
    q = rotationToQuaternion(R)
    aruco_Marker.pose.pose.orientation.x = q[0]
    aruco_Marker.pose.pose.orientation.y = q[1]
    aruco_Marker.pose.pose.orientation.z = q[2]
    aruco_Marker.pose.pose.orientation.w = q[3]

    ## For compatibility with gazebo

    aruco_MarkerList.markers.append(aruco_Marker)
    print(aruco_Marker.pose.pose.orientation)
    print(aruco_Marker.pose.pose.position)

for marker in aruco_MarkerList.markers:
    id = int(marker.id)
    time = marker.header.stamp
    # print(marker.pose.pose.position.x)
    # print(marker.pose.covariance)
    # print(marker)



a = [[-1.52919806,2.30157803,-1.21385095]]

# print(a[0][1])

# b = a[0][1]
# print(b)


# Define calibration filename
calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml"

## Load coefficients
def load_coefficient(calibration_file):
    with open(calibration_file) as stream:
        try:
            data_loaded = yaml.load(stream)
            c_matrix = np.array(data_loaded["camera_matrix"]["data"]).reshape(3,3)
            dist_coeff = np.array(data_loaded["distortion_coefficients"]["data"])
            return c_matrix, dist_coeff
        except yaml.YAMLError as exc:
            print(exc)

c_matrix, dist = load_coefficient(calibration_file)
print(c_matrix)
print(dist_coeff)