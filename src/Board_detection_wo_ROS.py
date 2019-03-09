#!/usr/bin/env python


import numpy as np
import yaml

import cv2
import math




# Static transformation from board to marker
x_tr = 0
y_tr = 0
z_tr = 0

## Generate dictionary
# dictionary_b = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
dictionary_b = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# dictionary_b = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(4,3,0.0633,0.0475,dictionary_b)# Doma
# board = cv2.aruco.CharucoBoard_create(8,5,.034566,.01725,dictionary_b)
# board = cv2.aruco.CharucoBoard_create(3,2,0.0895,0.0714,dictionary_b) # mechlab


# Define calibration filename
calibration_file = "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml"



## Define Aruco Params
arucoParams = cv2.aruco.DetectorParameters_create()
arucoParams.minMarkerPerimeterRate = 0.02
arucoParams.minCornerDistanceRate = 0.04
arucoParams.cornerRefinementMethod = 0
arucoParams.cornerRefinementMaxIterations = 35
arucoParams.markerBorderBits = 1
arucoParams.maxErroneousBitsInBorderRate = 0.4


## Initialization
markerIds = np.array([])
markerCorners = np.array([])
rejectedImgPoints = np.array([])
markerLength = 0.088
axis = np.float32([[markerLength/2,markerLength/2,0], [-markerLength/2,markerLength/2,0], [-markerLength/2,-markerLength/2,0], [markerLength/2,-markerLength/2,0],
                   [markerLength/2,markerLength/2,markerLength],[-markerLength/2,markerLength/2,markerLength],[-markerLength/2,-markerLength/2,markerLength],[markerLength/2,-markerLength/2,markerLength] ])


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


########## TCP ################
def aruco_detect(camera_matrix, dist_coeff):
    # cap = cv2.VideoCapture('tcpclientsrc host=ubiquityrobot port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    cap.set(cv2.CAP_PROP_FOCUS,0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE,3)
    # cap.set(cv2.CAP_PROP_SETTINGS,0)
    # cap.set(cv2.CAP_PROP_FOURCC =
    

    while(cap.isOpened()):
        ret, frame = cap.read()
        # frame = cv2.flip(frame,0)



        if ret==True:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            try:
                markerCorners_b,markerIds_b,_ = cv2.aruco.detectMarkers(gray,dictionary_b,parameters = arucoParams, cameraMatrix = camera_matrix,
            distCoeff = dist_coeff )

            except:
                print("crashed")
                markerCorners = []


        if len(markerCorners_b) > 0:

            try:
                ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(markerCorners_b, markerIds_b, gray, board,camera_matrix, dist_coeff)
                ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, camera_matrix, dist_coeff) # For a single marker
            
            except:
                ret = 0
                print("crashed")

            if ret > 0:


                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeff, rvec, tvec, 0.1)


                surface = cv2.contourArea(ch_corners, False)

                # All coordinates are in marker frame


            # frame = cv2.flip(frame,0)	
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



    cap.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    camera_matrix, dist_coeff = load_coefficient(calibration_file)
    aruco_detect(camera_matrix,dist_coeff)
    # rospy.spin()