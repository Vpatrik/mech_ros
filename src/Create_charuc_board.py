import cv2

# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

board = cv2.aruco.CharucoBoard_create(4,3,0.08,0.06,dictionary) # Doma
# board = cv2.aruco.CharucoBoard_create(3,2,0.10,0.08,dictionary) # Mechlab



aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

markerLength = 0.088

# img = board.draw((600, 500), boardImage, 10, 1 )
# img = board.draw((720, 540))
img = board.draw((1200, 900))
cv2.imwrite('charuco.png',img)

img_a = cv2.aruco.drawMarker(aruco_dict, 0, 250)
cv2.imwrite('aruco.png',img_a)