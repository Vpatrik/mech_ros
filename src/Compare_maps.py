#!/usr/bin/env python


import numpy as np
import cv2
import rospy
import math




def rotate_image(image, angle):
    diagonal = int(math.sqrt(image.shape[0]**2 + image.shape[1]**2))
    offset_y = (diagonal - image.shape[0])/2
    offset_x = (diagonal - image.shape[1])/2
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

def combine_two_color_images_with_anchor(image1, image2, anchor_y, anchor_x):
    foreground, background = image1.copy(), image2.copy()
    # Check if the foreground is inbound with the new coordinates and raise an error if out of bounds
    background_height = background.shape[0]
    background_width = background.shape[1]
    foreground_height = foreground.shape[0]
    foreground_width = foreground.shape[1]
    if foreground_height+anchor_y > background_height or foreground_width+anchor_x > background_width:
        raise ValueError("The foreground image exceeds the background boundaries at this location")

    alpha =0.5

    # do composite at specified location
    start_y = anchor_y
    start_x = anchor_x
    end_y = anchor_y+foreground_height
    end_x = anchor_x+foreground_width
    blended_portion = cv2.addWeighted(foreground,
                alpha,
                background[start_y:end_y, start_x:end_x],
                1 - alpha,
                0,
                background)
    background[start_y:end_y, start_x:end_x] = blended_portion
    cv2.imshow('composited image', background)

    if cv2.waitKey(0) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

filename_1 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_ground_truth.pgm'
# filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_1_transformed.pgm'
filename_2 = '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLab_2.pgm'

ground_truth = cv2.imread(filename_1,cv2.IMREAD_UNCHANGED)
created_map = cv2.imread(filename_2,cv2.IMREAD_UNCHANGED)




binary_map = mapToBinary(created_map)
binary_ground_truth = mapToBinary(ground_truth)


template = bbox2(binary_ground_truth)
cropped_map = bbox2(binary_map)

diagonal = int(math.sqrt(template.shape[0]**2 + template.shape[1]**2))
after_rotation = np.zeros((diagonal, diagonal), np.uint8)
padded_cropped_map = addPadding(cropped_map, after_rotation, centered=True)
rotated_template = rotate_image(template, 0)

# combine_two_color_images_with_anchor(rotated_template, binary_map,80,100)

img_display = padded_cropped_map.copy()
# img_display = binary_map.copy()


# # result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
# result = cv2.matchTemplate(binary_map, rotated_template, cv2.TM_SQDIFF)
# cv2.normalize( result, result, 0, 1, cv2.NORM_MINMAX, -1 )
# _minVal, _maxVal, minLoc, maxLoc = cv2.minMaxLoc(result, None)
# matchLoc = minLoc
# cv2.rectangle(img_display, matchLoc, (matchLoc[0] + rotated_template.shape[0], matchLoc[1] + rotated_template.shape[1]), 100, 2, 8, 0 )
# cv2.rectangle(result, matchLoc, (matchLoc[0] + rotated_template.shape[0], matchLoc[1] + rotated_template.shape[1]), 100, 2, 8, 0 )

# image_window = "Source Image"
# result_window = "Result window"
# cv2.imshow(image_window, img_display)
# cv2.imshow(result_window, result)

best_value = 1e10

for i in range(360):
    rotated_template = rotate_image(template, i)
    result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
    # result = cv2.matchTemplate(binary_map, rotated_template, cv2.TM_SQDIFF)
    # cv2.normalize( result, result, 0, 1, cv2.NORM_MINMAX, -1 )
    _minVal, _maxVal, minLoc, maxLoc = cv2.minMaxLoc(result, None)
    # print(i,_minVal)
    if _minVal < best_value:
        best_value = _minVal
        best_coord = minLoc
        best_angle = i

rotated_template = rotate_image(template, best_angle)
coord = (best_coord[0] + rotated_template.shape[0], best_coord[1] + rotated_template.shape[1])
cv2.rectangle(img_display, best_coord, coord, 100, 2, 8, 0 )
coord_center = (best_coord[0] + rotated_template.shape[0]/2, best_coord[1] + rotated_template.shape[1]/2)
print(best_angle)
print(coord_center)

rotated_template = rotate_image(template, best_angle)

rotated_template_padded = addPadding(rotated_template, padded_cropped_map, centered= False, origin= best_coord)
blended_img, blended_img2 = blendAndDrawDifference(rotated_template_padded, padded_cropped_map,0.5)

# combine_two_color_images_with_anchor(rotated_template, binary_map ,best_coord[1],best_coord[0])
# combine_two_color_images_with_anchor(rotated_template, padded_cropped_map ,best_coord[1],best_coord[0])

# image_window = "Source Image"
# cv2.imshow(image_window, img_display)


# ground_truth_binary = ~cv2.inRange(ground_truth, 180, 230)
# indices = np.argwhere(background_subtract)
# (ystart, xstart), (ystop, xstop) = indices.min(0), indices.max(0) + 1 
# template = background_subtract[ystart:ystop, xstart:xstop]

# affine_transform,_ = cv2.estimateAffine2D(ground_truth, transformed,)
# rigid_transform = cv2.estimateRigidTransform(ground_truth, transformed,False)

# print(affine_transform)
# print(rigid_transform)

# ground_truth_contours, contours, hierarchy = cv2.findContours(ground_truth_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# cv2.drawContours(ground_truth_contours, contours, -1, (100), 2)
# M = cv2.moments(contours[0])
# x = M['m10']/M['m00']
# y = M['m01']/M['m00']

# print(x,y)


rotated_template = rotate_image(template, 40)
cv2.namedWindow('rotated_template',cv2.WINDOW_NORMAL)
cv2.resizeWindow('transformed', 500,500)
cv2.imshow('rotated_template',blended_img)

cv2.namedWindow('cropped_map',cv2.WINDOW_NORMAL)
cv2.resizeWindow('transformed', 500,500)
cv2.imshow('cropped_map',padded_cropped_map)

cv2.namedWindow('blended',cv2.WINDOW_NORMAL)
cv2.resizeWindow('blended', 500,500)
cv2.imshow('blended',blended_img2)

# i = 0
# for s in range(20):
#     rotated_template = rotate_image(template, i)
#     i += 360/20
#     cv2.imshow('rotated_template',rotated_template)
#     if cv2.waitKey(500) & 0xFF == ord('q'):
#         cv2.destroyAllWindows()

if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()


# if __name__ == '__main__':
#     rospy.init_node('aruco_detect', anonymous=True)
#     marker_publisher = rospy.Publisher(marker_detector_topic, MarkerList,queue_size=10)
#     camera_matrix, dist_coeff = load_coefficient(calibration_file)
#     aruco_detect(camera_matrix,dist_coeff)
#     # rospy.spin()
