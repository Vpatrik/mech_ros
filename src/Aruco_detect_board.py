#!/usr/bin/env python


import numpy as np
# import cv2
import yaml
# import time
import rospy
import cv2
import math
import csv


from geometry_msgs.msg import PoseWithCovarianceStamped
from mech_ros_msgs.msg import MarkerList
from mech_ros_msgs.msg import Marker

filename = "/home/patrik/catkin_ws/src/mech_ros/localization/Data_natoceni.csv"


# Static transformation from board to marker
x_tr = 0
y_tr = 0
z_tr = 0

## Generate dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
dictionary_b = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# dictionary_b = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(4,3,0.0633,0.0475,dictionary_b)# Doma
# board = cv2.aruco.CharucoBoard_create(8,5,.034566,.01725,dictionary_b)
# board = cv2.aruco.CharucoBoard_create(3,2,0.0895,0.0714,dictionary_b) # mechlab

# Define topics
marker_detector_topic = "/markers"
board_detector_topic = "/board"
frame_id = "/camera"

# Define calibration filename
calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml"

markerLength = 0.088


## Define Aruco Detector Params
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
arucoParams.minMarkerPerimeterRate = 0.1 # min_mark_perim[pix]= max. dimension of image [pix]*minMark-per-Rate
arucoParams.maxMarkerPerimeterRate = 4.0
arucoParams.minOtsuStdDev = 5.0
arucoParams.perspectiveRemoveIgnoredMarginPerCell = 0.13
arucoParams.perspectiveRemovePixelPerCell = 8
arucoParams.polygonalApproxAccuracyRate = 0.01
arucoParams.markerBorderBits = 1
arucoParams.maxErroneousBitsInBorderRate = 0.04
arucoParams.minDistanceToBorder = 3


## Define Aruco board Detector Params
arucoParams_b = cv2.aruco.DetectorParameters_create()
arucoParams_b.adaptiveThreshConstant = 7
arucoParams_b.adaptiveThreshWinSizeMax = 45 # default 23
arucoParams_b.adaptiveThreshWinSizeMin = 3 # default 3
arucoParams_b.adaptiveThreshWinSizeStep = 7 # default 10
arucoParams_b.cornerRefinementMethod = 0
# arucoParams_b.errorCorrectionRate = 0.6
# arucoParams_b.minOtsuStdDev = 5.0
# arucoParams_b.perspectiveRemoveIgnoredMarginPerCell = 0.13
# arucoParams_b.perspectiveRemovePixelPerCell = 8
# arucoParams_b.polygonalApproxAccuracyRate = 0.01
arucoParams_b.markerBorderBits = 1
# arucoParams_b.maxErroneousBitsInBorderRate = 0.04
# arucoParams_b.minDistanceToBorder = 3



## Initialization
markerIds = np.array([])
markerCorners = np.array([])
rejectedImgPoints = np.array([])
axis = np.float32([[markerLength/2,markerLength/2,0], [-markerLength/2,markerLength/2,0], [-markerLength/2,-markerLength/2,0], [markerLength/2,-markerLength/2,0],
                   [markerLength/2,markerLength/2,markerLength],[-markerLength/2,markerLength/2,markerLength],[-markerLength/2,-markerLength/2,markerLength],[markerLength/2,-markerLength/2,markerLength] ])

## Initialization for statistics
marker_pose = np.zeros((50,3))
board_pose = np.zeros((50,3))
pose_difference = np.zeros((50,3))
surface_matrix = np.zeros((50,2))


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



# i = 0
# time_last = time.time()

############### Capture stream ###############

######## UDP ####################
#pipe = "udpsrc port=5777 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"

########## TCP ################
def aruco_detect(camera_matrix, dist_coeff):
    cap = cv2.VideoCapture('tcpclientsrc host=mechros1.local port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)
    # cap = cv2.VideoCapture(0)
    # pipe = 'tcp://10.42.0.85:5001'
    # cap = cv2.VideoCapture(pipe)


    # cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    cap.set(cv2.CAP_PROP_FOCUS,0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE,3)
    # cap.set(cv2.CAP_PROP_SETTINGS,0)
    # cap.set(cv2.CAP_PROP_FOURCC =

    k=0
    flag_started = False

    while(cap.isOpened()) and not(rospy.is_shutdown()):
        ret, frame = cap.read()
        # frame = cv2.flip(frame,0)
        time = rospy.Time.now()
        marker_yaw = None
        ret_b = 0

        if ret==True:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            markerCorners,markerIds,_ = cv2.aruco.detectMarkers(gray,dictionary,parameters = arucoParams, cameraMatrix = camera_matrix,
            distCoeff = dist_coeff )

            markerCorners_b,markerIds_b,_ = cv2.aruco.detectMarkers(gray,dictionary_b,parameters = arucoParams_b, cameraMatrix = camera_matrix,
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
                marker_yaw = Euler[2]

                # Coordinate vector of camera position from marker in camera coordinate frame
                aruco_Marker.pose.position.x = -tvec[i,0,2]
                aruco_Marker.pose.position.y = tvec[i,0,0]
                aruco_Marker.pose.position.z = tvec[i,0,1]
                # For comparsion with board
                marker_x = -tvec[i,0,2]
                marker_y = tvec[i,0,0]
                ## For compatibility with gazebo
                aruco_Marker.header.stamp = time

                # All coordinates are in marker frame
                aruco_MarkerList.markers.append(aruco_Marker)

            marker_publisher.publish(aruco_MarkerList)


        if len(markerCorners_b) > 0:
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(markerCorners_b, markerIds_b, gray, board,camera_matrix, dist_coeff)
            cv2.aruco.drawDetectedMarkers(frame,markerCorners_b, markerIds_b)
            if ret > 3:
                ret_b, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, camera_matrix, dist_coeff) # For a single marker
                
                if ret_b:
                        # cv2.aruco.drawDetectedMarkers(frame,markerCorners_b, markerIds_b)
                    aruco_MarkerList_b = MarkerList()
                    aruco_MarkerList_b.header.stamp = time
                    aruco_MarkerList_b.header.frame_id = frame_id

                    # cv2.aruco.drawAxis(frame, camera_matrix, dist_coeff, rvec[i], tvec[i], 0.1)
                    # imgpts, jac = cv2.projectPoints(axis, rvec[i], tvec[i], camera_matrix, dist_coeff)
                    # frame = draw(frame, markerCorners[i], imgpts)

                    # Fill MarkerList with each marker
                    aruco_Marker_b = Marker()
                    # aruco_Marker.id = str(markerIds[0][0])
                    # aruco_Marker.surface = surface

                    # Prevedeni rodrigues vectoru na rotacni matici
                    Rmat = np.zeros(shape=(3,3))
                    cv2.Rodrigues(rvec,Rmat)



                    # Convert from Opencv frame to ROS frame in camera
                    R = np.dot(R_ROS_O_camera, Rmat)

                    # Convert inverted matrix from Opencv frame to ROS frame in marker
                    R = np.dot(R, R_O_ROS_marker)

                    # Convert from Rotation matrix to Euler angles
                    Euler_b = rotationMatrixToEulerAngles(R.T) # rotace z markeru do kamery

                
                    # Fill Marker orientation vector
                    aruco_Marker_b.pose.orientation.y = Euler_b[2]
                    aruco_Marker_b.pose.orientation.r = Euler_b[0]
                    aruco_Marker_b.pose.orientation.p = Euler_b[1]

                    surface = cv2.contourArea(ch_corners, False)
                    aruco_Marker_b.surface = surface
                    # Surface - here used for display difference between board ang marker yaw
                    # if marker_yaw is not None:


                    #     if Euler[2] > math.pi/2:
                    #         Euler[2]-=math.pi
                    #     else:
                    #         Euler[2]+=math.pi
                    #     if marker_yaw > math.pi/2:
                    #         marker_yaw-=math.pi
                    #     else:
                    #         marker_yaw+=math.pi

                        # aruco_Marker.surface = (Euler[2] - marker_yaw)*180/(math.pi)


                        # aruco_Marker.pose.orientation.r = -tvec[2] - x_tr - marker_x
                        # aruco_Marker.pose.orientation.p = tvec[0] - y_tr - marker_y


                    # Coordinate vector of camera position from marker in camera coordinate frame
                    aruco_Marker_b.pose.position.x = -tvec[2]
                    aruco_Marker_b.pose.position.y = tvec[0]
                    aruco_Marker_b.pose.position.z = tvec[1]

                    ## For compatibility with gazebo
                    aruco_Marker_b.header.stamp = time

                    # All coordinates are in marker frame
                    aruco_MarkerList_b.markers.append(aruco_Marker_b)

                    board_publisher.publish(aruco_MarkerList_b)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            flag_started = True
            rospy.loginfo("Measurement started!")

        if k < 50 and flag_started:
            if len(markerCorners_b) > 0 and ret_b > 0:
                yaw_m = Euler[2] % (2*math.pi)
                yaw_b = Euler_b[2] % (2*math.pi)

                marker_pose[k,:] = aruco_Marker.pose.position.x,aruco_Marker.pose.position.y, yaw_m
                board_pose[k,:] = aruco_Marker_b.pose.position.x,aruco_Marker_b.pose.position.y, yaw_b
                surface_matrix[k,:] = aruco_Marker.surface, aruco_Marker_b.surface
                k +=1

        elif k >= 50 and flag_started:
            k = 0
            flag_started = False

            avg_pose_m = np.mean(marker_pose, axis= 0)
            avg_pose_b = np.mean(board_pose, axis= 0)
            avg_surface = np.mean(surface_matrix,axis=0)
            pose_difference = marker_pose - avg_pose_b
            measur_cov = np.cov(marker_pose,y = None, rowvar=False)
            # diff_cov = np.cov(pose_difference,y = None, rowvar=False)
            err_cov = np.sum(pose_difference*pose_difference, axis=0)/50

            

            # Write to file
            # x_m, y_m, Yaw_m, x_b, y_b, Yaw_b, var_meas_x, var_meas_y, var_meas_Yaw, var_err_x, var_err_y, var_err_Yaw
            row = [avg_surface[0], avg_surface[1], avg_pose_m[0],avg_pose_m[1], avg_pose_m[2],avg_pose_b[0],avg_pose_b[1], avg_pose_b[2],
            measur_cov[0,0],measur_cov[1,1], measur_cov[2,2], err_cov[0], err_cov[1], err_cov[2]]

            with open(filename, 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
            csvFile.close()

            rospy.loginfo("Measurement ended!")

            # Write to file
            # Matrix_cov_meas, Matrix_cov_err
            

        # frame = cv2.flip(frame,0)	
        cv2.imshow('frame',frame)
        if key == ord('q'):
            break

        # rospy.spin()

    # rospy.spin()

    csvFile.close()
    cap.release()
    #out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('aruco_detect', anonymous=True)
    marker_publisher = rospy.Publisher(marker_detector_topic, MarkerList,queue_size=10)
    board_publisher = rospy.Publisher(board_detector_topic, MarkerList,queue_size=10)
    camera_matrix, dist_coeff = load_coefficient(calibration_file)
    aruco_detect(camera_matrix,dist_coeff)
    # rospy.spin()