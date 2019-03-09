#!/usr/bin/env python

# import cv2

# cap = cv2.VideoCapture("cam_1")

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#        # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

########### TCP  - ZMQ ####################

# import cv2
# import zmq
# import base64
# import numpy as np

# context = zmq.Context()
# footage_socket = context.socket(zmq.SUB)
# footage_socket.bind('tcp://*:5555')
# footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

# while True:
#     try:
#         frame = footage_socket.recv_string()
#         img = base64.b64decode(frame)
#         npimg = np.fromstring(img, dtype=np.uint8)
#         source = cv2.imdecode(npimg, 1)
#         cv2.imshow("Stream", source)
#         cv2.waitKey(1)

#     except KeyboardInterrupt:
#         cv2.destroyAllWindows()
#         break

############## TCP - direct ###############

# import cv2

# cap = cv2.VideoCapture("tcp://10.42.0.1:8080")

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#        # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()


############## TCP - netcat ###############

# import cv2

# cap = cv2.VideoCapture("tcp://10.42.0.1:8080")

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#        # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

#######x GSTREAMER TCP ###########xx


# import cv2

# # cap = cv2.VideoCapture('tcpclientsrc host=10.42.0.1 port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)

# ## gst-launch-1.0 -v tcpclientsrc host=serverIp port=5000 \
# ## ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

# cap = cv2.VideoCapture('tcpclientsrc host=ubiquityrobot port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)


# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#        # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

import math
import numpy as np

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


pi = math.pi
R = eulerAnglesToRotationMatrix([pi/2,0,pi/2])
print(R)