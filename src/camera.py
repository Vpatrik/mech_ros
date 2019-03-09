import cv2

# cap = cv2.VideoCapture('tcpclientsrc host=ubiquityrobot port=8080  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
cap.set(cv2.CAP_PROP_FOCUS,0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
# cap.set(cv2.CAP_PROP_BUFFERSIZE,3)
# cap.set(cv2.CAP_PROP_SETTINGS,1)
# cap.set(cv2.CAP_PROP_SETTINGS,0.0)

while(cap.isOpened()):
    ret, frame = cap.read()

            # frame = cv2.flip(frame,0)	
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # rospy.spin()
cap.release()
#out.release()
cv2.destroyAllWindows()


# cams_test = 10
# for i in range(cams_test):
#     cap = cv2.VideoCapture(i)
#     test, frame = cap.read()
#     print("i : "+str(i)+" /// result: "+str(test))
