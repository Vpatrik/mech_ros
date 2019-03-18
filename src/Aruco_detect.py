#!/usr/bin/env python


import numpy as np
import cv2
import yaml
import rospy
import math


from geometry_msgs.msg import PoseWithCovarianceStamped
from mech_ros_msgs.msg import MarkerList
from mech_ros_msgs.msg import Marker

## Generate dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

# Define topics
marker_detector_topic = "/markers"
frame_id = "front_camera_link"

# Define calibration filename
calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml"
# calibration_file = "/home/patrik/catkin_ws/src/mech_ros/map/camCal2.npz"



## Define Aruco Detector Params
markerLength = 0.1118
arucoParams = cv2.aruco.DetectorParameters_create()

arucoParams.adaptiveThreshConstant = 7
arucoParams.adaptiveThreshWinSizeMax = 35 # default 23
arucoParams.adaptiveThreshWinSizeMin = 3 # default 3
arucoParams.adaptiveThreshWinSizeStep = 8 # default 10

arucoParams.cornerRefinementMethod = 1
arucoParams.cornerRefinementMaxIterations = 30
arucoParams.cornerRefinementMinAccuracy = 0.01
arucoParams.cornerRefinementWinSize = 5

arucoParams.errorCorrectionRate = 0.6
arucoParams.minCornerDistanceRate = 0.05 # min distance between marker corners,
# min. distance[pix] = Perimeter*minCornerDistanceRate
arucoParams.minMarkerDistanceRate = 0.05 # min distance between corners of different markers,
# min. distance[pix] = Perimeter(smaller one)*minMarkerDistanceRate
arucoParams.minMarkerPerimeterRate = 0.1
arucoParams.maxMarkerPerimeterRate = 4.0
arucoParams.minOtsuStdDev = 5.0
arucoParams.perspectiveRemoveIgnoredMarginPerCell = 0.13
arucoParams.perspectiveRemovePixelPerCell = 8
arucoParams.polygonalApproxAccuracyRate = 0.01
arucoParams.markerBorderBits = 1
arucoParams.maxErroneousBitsInBorderRate = 0.04
arucoParams.minDistanceToBorder = 3



## Initialization
markerIds = np.array([])
markerCorners = np.array([])
rejectedImgPoints = np.array([])
axis = np.float32([[markerLength/2,markerLength/2,0], [-markerLength/2,markerLength/2,0], [-markerLength/2,-markerLength/2,0], [markerLength/2,-markerLength/2,0],
                   [markerLength/2,markerLength/2,markerLength],[-markerLength/2,markerLength/2,markerLength],[-markerLength/2,-markerLength/2,markerLength],[markerLength/2,-markerLength/2,markerLength] ])




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

    # with open(calibration_file) as stream:
    #     try:
    #         data_loaded = yaml.load(stream)
    #         c_matrix = np.array(data_loaded["camera_matrix"]["data"]).reshape((3,3))
    #         dist_coeff = np.array(data_loaded["distortion_coefficients"]["data"])
    #         return c_matrix, dist_coeff
    #     except yaml.YAMLError as exc:
    #         print(exc)

    with np.load(calibration_file) as X:
        c_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    return c_matrix, dist_coeff

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


############### Capture stream ###############

######## UDP ####################
#pipe = "udpsrc port=5777 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"

########## TCP ################
def aruco_detect(camera_matrix, dist_coeff):
    cap = cv2.VideoCapture('tcpclientsrc host=mechros1.local port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)
    # cap = cv2.VideoCapture(0)
    # pipe = "tcpclientsrc host=10.42.0.136 port=8080 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"
    # pipe = 'tcp://10.42.0.85:5001'
    # cap = cv2.VideoCapture(pipe)    

    while(cap.isOpened()) and not(rospy.is_shutdown()):
        ret, frame = cap.read()
        # frame = cv2.flip(frame,0)
        time = rospy.Time.now()
        latency =  rospy.Duration(0,int(2.2e8))
        time = time - latency

        if ret==True:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            markerCorners,markerIds,rejectedImgPoints = cv2.aruco.detectMarkers(gray,dictionary,parameters = arucoParams, cameraMatrix = camera_matrix,
            distCoeff = dist_coeff )


        if len(markerCorners) > 0:
            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerLength, camera_matrix, dist_coeff) # For a single marker
            cv2.aruco.drawDetectedMarkers(frame,markerCorners, markerIds)
            aruco_MarkerList = MarkerList()
            aruco_MarkerList.header.stamp = time
            aruco_MarkerList.header.frame_id = frame_id


            for i in range(markerIds.size):
                frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeff, rvec[i], tvec[i], 0.1)
                imgpts, jac = cv2.projectPoints(axis, rvec[i], tvec[i], camera_matrix, dist_coeff)
                frame = draw(frame, markerCorners[i], imgpts)

                # Calculate surface area in pixels
                surface = cv2.contourArea(markerCorners[i], False)
                
                # Fill MarkerList with each marker
                aruco_Marker = Marker()
                aruco_Marker.id = str(markerIds[i,0])
                aruco_Marker.surface = surface

                # Prevedeni rodrigues vectoru na rotacni matici
                Rmat = np.zeros(shape=(3,3))
                cv2.Rodrigues(rvec[i,0],Rmat)

                # Convert from Opencv frame to ROS frame in camera
                R = np.dot(R_ROS_O_camera, Rmat)

                # Convert inverted matrix from Opencv frame to ROS frame in marker
                R = np.dot(R, R_O_ROS_marker)

                # Convert from Rotation matrix to Euler angles
                Euler = rotationMatrixToEulerAngles(R.T) # rotace z markeru do kamery

                # Fill Marker orientation vector
                aruco_Marker.pose.orientation.r = Euler[0]
                aruco_Marker.pose.orientation.p = Euler[1]
                aruco_Marker.pose.orientation.y = Euler[2]


                # Coordinate vector of camera position from marker in camera coordinate frame
                aruco_Marker.pose.position.x = -tvec[i,0,2]
                aruco_Marker.pose.position.y = tvec[i,0,0]
                aruco_Marker.pose.position.z = -tvec[i,0,1]

                ## For compatibility with gazebo
                aruco_Marker.header.stamp = time

                # All coordinates are in marker frame
                aruco_MarkerList.markers.append(aruco_Marker)

            marker_publisher.publish(aruco_MarkerList)

            # frame = cv2.flip(frame,0)	
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # rospy.spin()

    cap.release()
    #out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('aruco_detect', anonymous=True)
    marker_publisher = rospy.Publisher(marker_detector_topic, MarkerList,queue_size=10)
    camera_matrix, dist_coeff = load_coefficient(calibration_file)
    aruco_detect(camera_matrix,dist_coeff)
    # rospy.spin()
