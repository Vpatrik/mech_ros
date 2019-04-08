#!/usr/bin/env python


import numpy as np
import cv2
import rospy
import math

import time


def rotate_image(image, angle, create_rot_image= True, rot_image= None, border_value= None):
    diagonal = int(math.sqrt(image.shape[0]**2 + image.shape[1]**2))
    offset_y = (diagonal - image.shape[0])/2
    offset_x = (diagonal - image.shape[1])/2
    if create_rot_image:
        rot_image = np.zeros((diagonal, diagonal), dtype='uint8')
    image_center = (diagonal/2, diagonal/2)

    R = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rot_image[offset_y:(offset_y + image.shape[0]), \
            offset_x:(offset_x + image.shape[1])] = image
    if border_value != None:
        rotated_image = cv2.warpAffine(rot_image, R, (diagonal, diagonal), flags=cv2.INTER_LINEAR,
                                            borderValue= border_value)
        return rotated_image
    rotated_image = cv2.warpAffine(rot_image, R, (diagonal, diagonal), flags=cv2.INTER_LINEAR)

    return rotated_image


def bbox2(img):
    rows = np.any(img, axis=1)
    cols = np.any(img, axis=0)
    ymin, ymax = np.where(rows)[0][[0, -1]]
    xmin, xmax = np.where(cols)[0][[0, -1]]
    return img[ymin:ymax+1, xmin:xmax+1], (ymin,xmin), (ymax, xmax)

def mapToBinary(img):
    # height = img.shape[0]
    # width = img.shape[1]
    img = ~cv2.inRange(img, 200, 210)
    # unknown_color_image = np.ones((height,width), np.uint8)*unknown_pixel_color
    # binary_map = (unknown_color_image - img)
    return img

def addPadding(to_pad, img_to_compare, centered = True, origin = None, color = 0):
    # Even if to_pad image is larger than template, padding has to be added because of sometimes
    # unavoidable padding on template
    if centered:
        height = int(max(to_pad.shape[0], img_to_compare.shape[0])*1.2)
        width = int(max(to_pad.shape[1], img_to_compare.shape[1])*1.2)
        padded_image = np.ones((height, width), dtype='uint8')*color
        offset_y = (height - to_pad.shape[0])/2
        offset_x = (width - to_pad.shape[1])/2
        padded_image[offset_y:(offset_y+to_pad.shape[0]),offset_x:(offset_x+to_pad.shape[1])] = to_pad
        return padded_image

    else:
        height = img_to_compare.shape[0]
        width = img_to_compare.shape[1]
        padded_image = np.ones((height, width), dtype='uint8')*color
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
    # combined = cv2.add(blended_img, difference)
    # Make the background from black to gray
    # _,combined2 = cv2.threshold(cv2.cvtColor(combined, cv2.COLOR_BGR2GRAY, dstCn= 0),0, 255,
    #                                 cv2.THRESH_BINARY_INV)
    # combined2 = cv2.subtract(combined2,100)
    # combined2 = cv2.add(combined, cv2.cvtColor(combined2, cv2.COLOR_GRAY2BGR, dstCn= 0))

    return difference

sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)
unknown_pixel_color = 205
t = time.time()

filename_1 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_ground_truth.pgm'
filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_1_transformed.pgm'
# filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLab_2.pgm'

ground_truth = cv2.imread(filename_1,cv2.IMREAD_UNCHANGED)
created_map = cv2.imread(filename_2,cv2.IMREAD_UNCHANGED)

binary_map = mapToBinary(created_map)
binary_ground_truth = mapToBinary(ground_truth)


template,_,_ = bbox2(binary_ground_truth)
cropped_map,_,_ = bbox2(binary_map)

diagonal = int(math.sqrt(template.shape[0]**2 + template.shape[1]**2))
after_rotation = np.zeros((diagonal, diagonal), np.uint8)
padded_cropped_map = addPadding(cropped_map, after_rotation, centered=True)


best_value = 1e10
second_best = 1e10
division = 20
precision = 1
second_angle = 0
best_angle = 0
best_angle_refined = None


for i in range(360/division):
    rotated_template = rotate_image(template, i*division,create_rot_image= False, 
                                        rot_image= after_rotation)
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
    rotated_template = rotate_image(template, best_angle + signum*i*precision,create_rot_image= False,
                                        rot_image= after_rotation)
    result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
    min_val, _, min_loc, _ = cv2.minMaxLoc(result, None)
    if min_val < best_value:
        best_value = min_val
        best_coord = min_loc
        best_angle_refined = best_angle + signum*i*precision

if isinstance(best_angle_refined, type(None)):
    best_angle_refined = best_angle

print(best_angle_refined)

def getBorder(original, thresholded_wo_border, angle= None):
    # Make obstacle pixels 0
    _,original_inverted = cv2.threshold(original, 10, 255, cv2.THRESH_BINARY_INV)
    # Crop and rotate image if needed
    if angle != None:
        original_border,min_px,max_px = bbox2(original_inverted)
        original_border_rotated = rotate_image(original_border, angle)
    # Else crop both images and don't rotate
    else:
        thresholded_wo_border, min_px, max_px = bbox2(thresholded_wo_border)
        original_border_rotated = original_inverted[min_px[0]:max_px[0]+1, min_px[1]:max_px[1]+1]
    # Threshold after rotation - beacause of interpolation
    _,original_border_rotated_binary = cv2.threshold(original_border_rotated, 127, 255,cv2.THRESH_BINARY_INV)
    # Add border to binary image(only uknown pixels were 0 before this operation)
    # combined_rotated = cv2.bitwise_and(thresholded_wo_border, thresholded_wo_border,
    #                             mask = original_border_rotated_binary)
    return original_border_rotated_binary, min_px, max_px

rotated_template = rotate_image(template, best_angle_refined)
_,rotated_template = cv2.threshold(rotated_template, 127, 255,cv2.THRESH_BINARY)
# _,ground_truth_inverted = cv2.threshold(ground_truth, 10, 255, cv2.THRESH_BINARY_INV)
# template_border,_,_ = bbox2(ground_truth_inverted)
# template_border_rotated = rotate_image(template_border, best_angle_refined)
# _,template_border_rotated_binary = cv2.threshold(template_border_rotated, 127, 255,cv2.THRESH_BINARY_INV)
# template_combined_rotated = cv2.bitwise_and(rotated_template, rotated_template,
#                             mask = template_border_rotated_binary)
                            
alpha = 0.5
rotated_template_padded = addPadding(rotated_template, padded_cropped_map, centered= False, origin= best_coord)
blended_img = blendAndDrawDifference(rotated_template_padded,padded_cropped_map, 0.5)

border_t, min_px, max_px = getBorder(ground_truth, rotated_template, angle= best_angle_refined)
border_t = addPadding(border_t, padded_cropped_map, centered= False, origin= best_coord, color = 255)
border,_,_= getBorder(created_map, binary_map)
border = addPadding(border, after_rotation, centered=True, color = 255)
blended_img2 = blendAndDrawDifference(border_t,border, 0.5)

combined_differences =  cv2.bitwise_or(blended_img, blended_img2)
# Make mask from differences
_,combined_differences_mask= cv2.threshold(cv2.cvtColor(combined_differences, cv2.COLOR_BGR2GRAY, dstCn= 0),0, 255,
                                            cv2.THRESH_BINARY_INV)
# Crop, rotate, threshold and pad ground truth map
ground_truth_cropped = ground_truth[min_px[0]:max_px[0]+1, min_px[1]:max_px[1]+1]
diagonal = int(math.sqrt(ground_truth_cropped.shape[0]**2 + ground_truth_cropped.shape[1]**2))
rotation_image = np.ones((diagonal, diagonal), np.uint8)*unknown_pixel_color
ground_truth_rotated = rotate_image(ground_truth_cropped, best_angle_refined, create_rot_image= False,
                                         rot_image= rotation_image, border_value= unknown_pixel_color)
_,ground_truth_thresh = cv2.threshold(ground_truth_rotated, 180, 255,cv2.THRESH_TOZERO)
ground_truth_padded = addPadding(ground_truth_thresh, padded_cropped_map, centered= False, origin= best_coord,
                                    color = unknown_pixel_color)

# Mask ground truth template
template_masked = cv2.bitwise_and(ground_truth_padded, ground_truth_padded,
                            mask = combined_differences_mask)
# Finally convert template_mask to BGR
template_masked = cv2.cvtColor(template_masked, cv2.COLOR_GRAY2BGR, dstCn= 0)
result = cv2.add(template_masked, combined_differences)



###################################################
# Add padding for further combination of images
# combined_template_padded = addPadding(combined_template, padded_cropped_map, centered= False,
#                                         origin= best_coord)
# combined_map_padded = addPadding(combined_map, combined_template, centered=True)

# border_t = getBorder(ground_truth, rotated_template, angle= best_angle_refined)
# border = getBorder(created_map, binary_map)
# alpha = 0.5
# blended_img2 = cv2.addWeighted(border_t,alpha,border,1 - alpha,0, dtype= -1)

# Add occupied cells to images
# combined_template = addBorder2Map(ground_truth, rotated_template, angle= best_angle_refined)
# combined_map = addBorder2Map(created_map, binary_map)

# Add padding for further combination of images
# combined_template_padded = addPadding(combined_template, padded_cropped_map, centered= False,
#                                         origin= best_coord)
# combined_map_padded = addPadding(combined_map, combined_template, centered=True)

# Finally display differences
# blended_img, blended_img2 = blendAndDrawDifference(combined_template_padded, combined_map_padded,0.5)
# font = cv2.FONT_HERSHEY_COMPLEX
# cv2.putText(blended_img2,'95% of map is correct. Good job!',(20,30), font, 0.8,(100,255,180),1,cv2.LINE_AA)

cv2.namedWindow('Final',cv2.WINDOW_NORMAL)
cv2.resizeWindow('Final', 1000,1000)
cv2.imshow('Final',ground_truth_rotated)

cv2.namedWindow('border',cv2.WINDOW_NORMAL)
cv2.resizeWindow('border', 1000,1000)
cv2.imshow('border',template_masked)


cv2.namedWindow('intermediate',cv2.WINDOW_NORMAL)
cv2.resizeWindow('intermediate', 1000,1000)
cv2.imshow('intermediate',result)

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
