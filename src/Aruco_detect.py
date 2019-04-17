#!/usr/bin/env python

# Patrik Vavra 2019

"""
Node for detecting ArUco markers and publishing transformations of detected markers
after tranformation to ROS coordinate system.

"""




from numpy import array, zeros, dot, load, finfo
import cv2
from yaml import load, YAMLError
import rospy
from math import atan2, sqrt
from threading import Thread, Event

from geometry_msgs.msg import PoseWithCovarianceStamped
from mech_ros_msgs.msg import MarkerList
from mech_ros_msgs.msg import Marker


class VideoCapture(Thread):

    def __init__(self, ready):
        Thread.__init__(self)
        self.daemon = True
        self.ready = ready
        # self.cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture('tcpclientsrc host=mechros2.local port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)

        # ROS adjustable parameters
        image_width = rospy.get_param("~image_width", 1280)
        image_height = rospy.get_param("~image_height", 720)
        fps = rospy.get_param("~fps", 20)

        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.current_frame = None
        self.time = None
        self.initialized = False
        self.start()

        if not self.cap.isOpened():
            rospy.logerr("Cannot open stream")
            exit()
        else:
            rospy.loginfo("Stream OPENED")


    def run(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                self.time = rospy.Time.now()
                # Flip image and convert to grayscale
                self.current_frame = cv2.cvtColor(cv2.flip(frame,-1),cv2.COLOR_BGR2GRAY)
                if not(self.initialized):
                    self.ready.set()
                    self.initialized = True

    def get_current_frame(self):
        return self.current_frame, self.time

class Detector:
    
    def __init__(self):

        rospy.init_node("Aruco_detect")

        self.ready = Event()
        self.stream = VideoCapture(self.ready)

        self.c_matrix = None
        self.dist_coeff = None

        # ROS parameters
        self.marker_length = rospy.get_param("~marker_length", 0.118)
        marker_detector_topic = rospy.get_param("~markers_topic","/markers")
        self.frame_id = rospy.get_param("~camera_frame","front_camera_link")
        calibration_file = rospy.get_param("~calibration_file", "/home/patrik/catkin_ws/src/mech_ros/Config_ARuco/camera.yaml")
        calibration_file_type = rospy.get_param("~calibration_file_type","yaml")
        dictionary = rospy.get_param("~dictionary","DICT_5X5_50")


        # Define ArUco detector and dictionary
        try:
            self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        except:
            rospy.logerr("Wrongly defined ArUco dictionary!")

        arucoParams = cv2.aruco.DetectorParameters_create()
        arucoParams.adaptiveThreshConstant = 7
        #arucoParams.adaptiveThreshWinSizeMax = 27 # default 23
        #arucoParams.adaptiveThreshWinSizeMin = 3 # default 3
        #arucoParams.adaptiveThreshWinSizeStep = 8 # default 10

        ##arucoParams.cornerRefinementMethod = 1
        ##arucoParams.cornerRefinementMaxIterations = 30
        ##arucoParams.cornerRefinementMinAccuracy = 0.01
        ##arucoParams.cornerRefinementWinSize = 5

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

        self.arucoParams = arucoParams

        # Init publisher
        self.marker_publisher = rospy.Publisher(marker_detector_topic, MarkerList, queue_size=10)

        # Load camera calibration files
        self.load_calibration(calibration_file, calibration_file_type)


    def run_detection(self):

        # Matrix for conversion from ROS frame to OpenCV in camera
        R_ROS_O_camera = array([[  0.0,  0.0,   1.0],
        [  -1.0,   0.0,  0.0],
        [  0.0,   -1.0,   0.0]])

        # Matrix for conversion from OpenCV frame to ROS in marker
        R_O_ROS_marker = array([[  0.0,  1.0,   0.0],
        [  0.0,   0.0,  1.0],
        [  1.0,   0.0,   0.0]])

        # Wait until first frame is initialized
        self.ready.wait()

        while(self.stream.cap.isOpened()) and not(rospy.is_shutdown()):
            image, time = self.stream.get_current_frame()
            markerIds = array([])
            markerCorners = array([])

            markerCorners,markerIds,_ = cv2.aruco.detectMarkers(image, self.dictionary, parameters = self.arucoParams, cameraMatrix = self.c_matrix,
            distCoeff = self.dist_coeff )


            if len(markerCorners) > 0:
                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, self.marker_length, self.c_matrix, self.dist_coeff) # For a single marker
                aruco_MarkerList = MarkerList()
                aruco_MarkerList.header.stamp = time
                aruco_MarkerList.header.frame_id = self.frame_id


                for i in range(markerIds.size):

                    # Calculate surface area in pixels
                    surface = cv2.contourArea(markerCorners[i], False)
                    
                    # Fill MarkerList with each marker
                    aruco_Marker = Marker()
                    aruco_Marker.id = str(markerIds[i,0])
                    aruco_Marker.surface = surface

                    # Convert Rodrigues vector to rotation matrix
                    Rmat = zeros(shape=(3,3))
                    cv2.Rodrigues(rvec[i,0],Rmat)

                    # Convert from Opencv frame to ROS frame in camera
                    R = dot(R_ROS_O_camera, Rmat)

                    # Convert inverted matrix from Opencv frame to ROS frame in marker
                    R = dot(R, R_O_ROS_marker)

                    # Convert from Rotation matrix to Euler angles
                    Euler = self.rotationMatrixToEulerAngles(R.T) # Rotation from marker to camera

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

                self.marker_publisher.publish(aruco_MarkerList)

        self.stream.cap.release()
        

    ## Load coefficients
    def load_calibration(self, calibration_file, load_method = "yaml"):

        if load_method == "yaml":
            with open(calibration_file) as stream:
                try:
                    data_loaded = load(stream)
                    self.c_matrix = array(data_loaded["camera_matrix"]["data"]).reshape((3,3))
                    self.dist_coeff = array(data_loaded["distortion_coefficients"]["data"])
                    rospy.logdebug("Calibration files for camera loaded")
                except YAMLError as exc:
                    rospy.logerr(exc)

        elif load_method == "numpy":
            with load(calibration_file) as X:
                self.c_matrix, self.dist_coeff, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
                rospy.logdebug("Calibration files for camera loaded")
        
        else:
            rospy.logerr("Specify valid calibration_file_type. Valid types: 'yaml','numpy'")


    def rotationMatrixToEulerAngles(self, M, cy_thresh=None):
        # cy_thresh : None or scalar, optional
        #    threshold below which to give up on straightforward arctan for
        #    estimating x rotation.  If None (default), estimate from
        #    precision of input. Source : http://www.graphicsgems.org/
        _FLOAT_EPS_4 = finfo(float).eps * 4.0
        if cy_thresh is None:
            try:
                cy_thresh = finfo(M.dtype).eps * 4
            except ValueError:
                cy_thresh = _FLOAT_EPS_4
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
        # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
        cy = sqrt(r33*r33 + r23*r23)
        if cy > cy_thresh: # cos(y) not close to zero, standard form
            z = atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
            y = atan2(r13,  cy) # atan2(sin(y), cy)
            x = atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
        else: # cos(y) (close to) zero, so x -> 0.0 (see above)
            # so r21 -> sin(z), r22 -> cos(z) and
            z = atan2(r21,  r22)
            y = atan2(r13,  cy) # atan2(sin(y), cy)
            x = 0.0
        return [x, y, z]



if __name__ == '__main__':
    try:
        ArUcoDetector = Detector()
        ArUcoDetector.run_detection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
