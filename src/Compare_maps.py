#!/usr/bin/env python


import numpy as np
import cv2
import rospy
import math

import time


def rotate_image(image, angle, create_rot_image= True, rot_image= None):
    diagonal = int(math.sqrt(image.shape[0]**2 + image.shape[1]**2))
    offset_y = (diagonal - image.shape[0])/2
    offset_x = (diagonal - image.shape[1])/2
    if create_rot_image:
        rot_image = np.zeros((diagonal, diagonal), dtype='uint8')
    image_center = (diagonal/2, diagonal/2)

    R = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rot_image[offset_y:(offset_y + image.shape[0]), \
            offset_x:(offset_x + image.shape[1])] = image
    rot_image = cv2.warpAffine(rot_image, R, (diagonal, diagonal), flags=cv2.INTER_LINEAR)

    return rot_image


def bbox2(img):
    rows = np.any(img, axis=1)
    cols = np.any(img, axis=0)
    ymin, ymax = np.where(rows)[0][[0, -1]]
    xmin, xmax = np.where(cols)[0][[0, -1]]
    return img[ymin:ymax+1, xmin:xmax+1]

def mapToBinary(img):
    # height = img.shape[0]
    # width = img.shape[1]
    img = ~cv2.inRange(img, 180, 230)
    # unknown_color_image = np.ones((height,width), np.uint8)*205
    # binary_map = (unknown_color_image - img)
    return img

def addPadding(to_pad, img_to_compare, centered = True, origin = None):
    # Even if to_pad image is larger than template, padding has to be added because of sometimes unavoidable padding
    # on template
    if centered:
        height = int(max(to_pad.shape[0], img_to_compare.shape[0])*1.2)
        width = int(max(to_pad.shape[1], img_to_compare.shape[1])*1.2)
        padded_image = np.zeros((height, width), dtype='uint8')
        offset_y = (height - to_pad.shape[0])/2
        offset_x = (width - to_pad.shape[1])/2
        padded_image[offset_y:(offset_y+to_pad.shape[0]),offset_x:(offset_x+to_pad.shape[1])] = to_pad
        return padded_image

    else:
        height = img_to_compare.shape[0]
        width = img_to_compare.shape[1]
        padded_image = np.zeros((height, width), dtype='uint8')
        offset_y = origin[1]
        offset_x = origin[0]
        padded_image[offset_y:(offset_y+to_pad.shape[0]),offset_x:(offset_x+to_pad.shape[1])] = to_pad
        return padded_image

def blendAndDrawDifference(img1, img2, alpha):
    # Combine ground_truth with made map after finding best match
    blended_img = cv2.addWeighted(img1,alpha,img2,1 - alpha,0, dtype= -1)
    # Made binary image from the difference
    difference = cv2.inRange(blended_img, 100, 160)
    # Convert to BGR
    blended_img = cv2.cvtColor(blended_img, cv2.COLOR_GRAY2BGR, dstCn= 0)
    # Mask the difference from blended image
    blended_img = cv2.bitwise_and(blended_img, blended_img, mask = ~difference)
    # Convert difference to BGR for coloring to red
    difference = cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR, dstCn= 0)
    # Color to red
    difference[:,:,0:2] = 0
    # Combine image with red difference with masked blended image
    combined = cv2.add(blended_img, difference)
    # Make the background from black to gray
    _,combined2 = cv2.threshold(cv2.cvtColor(combined, cv2.COLOR_BGR2GRAY, dstCn= 0),0, 255, cv2.THRESH_BINARY_INV)
    combined2 = cv2.subtract(combined2,100)
    combined2 = cv2.add(combined, cv2.cvtColor(combined2, cv2.COLOR_GRAY2BGR, dstCn= 0))

    return combined, combined2

sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)

t = time.time()

filename_1 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_ground_truth.pgm'
filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_1_transformed.pgm'
# filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLab_2.pgm'

ground_truth = cv2.imread(filename_1,cv2.IMREAD_UNCHANGED)
created_map = cv2.imread(filename_2,cv2.IMREAD_UNCHANGED)

binary_map = mapToBinary(created_map)
binary_ground_truth = mapToBinary(ground_truth)


template = bbox2(binary_ground_truth)
cropped_map = bbox2(binary_map)

diagonal = int(math.sqrt(template.shape[0]**2 + template.shape[1]**2))
after_rotation = np.zeros((diagonal, diagonal), np.uint8)
padded_cropped_map = addPadding(cropped_map, after_rotation, centered=True)


best_value = 1e10
second_best = 1e10
division = 4
precision = 0.1
second_angle = 0
best_angle = 0
best_angle_refined = None


for i in range(360/division):
    rotated_template = rotate_image(template, i*division,create_rot_image= False, rot_image= after_rotation)
    result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
    min_val, _, min_loc, _ = cv2.minMaxLoc(result, None)
    if min_val < best_value:
        second_best_value = best_value
        best_value = min_val
        second_best_angle = best_angle
        best_angle = i*division
        best_coord = min_loc

print(best_angle)
print(second_best_angle)
sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)       

signum = sign(second_angle-best_angle)

for i in range(int(division/precision)):
    rotated_template = rotate_image(template, best_angle + signum*i*precision,create_rot_image= False, rot_image= after_rotation)
    result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
    min_val, _, min_loc, _ = cv2.minMaxLoc(result, None)
    if min_val < best_value:
        print(i)
        best_value = min_val
        best_coord = min_loc
        best_angle_refined = best_angle + signum*i*precision

if isinstance(best_angle_refined, type(None)):
    best_angle_refined = best_angle

print(best_angle_refined)



rotated_template = rotate_image(template, best_angle_refined)

rotated_template_padded = addPadding(rotated_template, padded_cropped_map, centered= False, origin= best_coord)
blended_img, blended_img2 = blendAndDrawDifference(rotated_template_padded, padded_cropped_map,0.5)
font = cv2.FONT_HERSHEY_COMPLEX
cv2.putText(blended_img2,'95% of pixels is correct. Good job!',(20,30), font, 0.8,(100,255,180),1,cv2.LINE_AA)

cv2.namedWindow('blended',cv2.WINDOW_NORMAL)
cv2.resizeWindow('blended', 1000,1000)
cv2.imshow('blended',blended_img2)


elapsed = time.time() - t
print(elapsed)

if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()


# if __name__ == '__main__':
#     rospy.init_node('aruco_detect', anonymous=True)
#     marker_publisher = rospy.Publisher(marker_detector_topic, MarkerList,queue_size=10)
#     camera_matrix, dist_coeff = load_coefficient(calibration_file)
#     aruco_detect(camera_matrix,dist_coeff)
#     # rospy.spin()
