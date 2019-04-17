#!/usr/bin/env python

# Patrik Vavra 2019

"""
Automatically compare ground truth map with map created from SLAM.
This functionality exposed as Service.
Visualization runs in seperate Thread to not latch Service server.
"""

import numpy as np
import cv2
import rospy
from math import sqrt
from threading import Thread, Event

from mech_ros_msgs.srv import Map_comparsion
from mech_ros_msgs.srv import Map_comparsionResponse


class Comparator:
    def __init__(self):

        rospy.init_node("map_comparator")

        # ROS adjustable parameters
        self.coarse_angle = rospy.get_param("~coarse_angle_self.coarse_angle", 6)
        self.refinement_angle = rospy.get_param("~angle_self.refinement_angle", 0.1)
        ground_truth = rospy.get_param("~ground_truth_map", '/home/patrik/catkin_ws/src/mech_ros/map/map_comparsion/MechLAB_ground_truth.pgm')
        
        rospy.Service('compare_maps',Map_comparsion, self.compare)

        try:
            self.ground_truth_img = cv2.imread(ground_truth,cv2.IMREAD_UNCHANGED)
        except:
            rospy.logerr('Specify correct ground truth map image!')

        # Prepare ground truth map
        self.template = self.adjust_map(self.ground_truth_img)

        # Create visualizer
        self.visualizer = Visualization()

    def compare(self, req):
        rospy.loginfo('Received request to compare maps!')
        # If not defined, used from node initialization
        if req.coarse_angle != 0:
            self.coarse_angle = req.coarse_angle
        if req.refinement_angle != 0:
            self.refinement_angle = req.refinement_angle

        sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)

        # Initialization
        best_value = 1e10
        second_best_angle = 0
        best_angle = 0
        best_angle_refined = None
        alpha = 0.5
        template = self.template
        created_map = cv2.imread(req.path_to_map,cv2.IMREAD_UNCHANGED)

        unknown_pixel_color = 205

        binary_map = self.mapToBinary(created_map)
        cropped_map,_,_ = self.bbox2(binary_map)
        diagonal = int(sqrt(template.shape[0]**2 + template.shape[1]**2))
        after_rotation = np.zeros((diagonal, diagonal), np.uint8)
        padded_cropped_map = self.addPadding(cropped_map, after_rotation, centered=True)

        # Match created map with rotated map with coarse angle
        for i in range(360/self.coarse_angle):
            rotated_template = self.rotate_image(template, i*self.coarse_angle,create_rot_image= False, 
                                                rot_image= after_rotation)
            result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
            min_val, _, min_loc, _ = cv2.minMaxLoc(result, None)
            if min_val < best_value:
                best_value = min_val
                second_best_angle = best_angle
                best_angle = i*self.coarse_angle
                best_coord = min_loc  

        # Refine rotation angle
        signum = sign(second_best_angle-best_angle)
        for i in range(int(self.coarse_angle/self.refinement_angle)):
            rotated_template = self.rotate_image(template, best_angle + signum*i*self.refinement_angle,create_rot_image= False,
                                                rot_image= after_rotation)
            result = cv2.matchTemplate(padded_cropped_map, rotated_template, cv2.TM_SQDIFF)
            min_val, _, min_loc, _ = cv2.minMaxLoc(result, None)
            if min_val < best_value:
                best_value = min_val
                best_coord = min_loc
                best_angle_refined = best_angle + signum*i*self.refinement_angle

        if isinstance(best_angle_refined, type(None)):
            best_angle_refined = best_angle

        # Rotate both images with refined angle and threshold them
        rotated_template = self.rotate_image(template, best_angle_refined)
        _,rotated_template = cv2.threshold(rotated_template, 127, 255,cv2.THRESH_BINARY)                               
        rotated_template_padded = self.addPadding(rotated_template, padded_cropped_map, centered= False, origin= best_coord)
        blended_img = self.blendAndDrawDifference(rotated_template_padded,padded_cropped_map, alpha)

        # Get obstacle pixels from ground truth as black
        border_t, min_px, max_px = self.getBorder(self.ground_truth_img, rotated_template, angle= best_angle_refined)
        border_t = self.addPadding(border_t, padded_cropped_map, centered= False, origin= best_coord, color = 255)
        
        # Get obstacle pixels from created map as black
        border,_,_= self.getBorder(created_map, binary_map)
        border = self.addPadding(border, after_rotation, centered=True, color = 255)
        blended_img2 = self.blendAndDrawDifference(border_t,border, alpha)
        
        # Combine differences in one image
        combined_differences =  cv2.bitwise_or(blended_img, blended_img2)
        
        # Make mask from differences
        _,combined_differences_mask= cv2.threshold(cv2.cvtColor(combined_differences, cv2.COLOR_BGR2GRAY, dstCn= 0),0, 255,
                                                    cv2.THRESH_BINARY_INV)

        # Crop, rotate, threshold and pad ground truth map
        ground_truth_cropped = self.ground_truth_img[min_px[0]:max_px[0]+1, min_px[1]:max_px[1]+1]
        diagonal = int(sqrt(ground_truth_cropped.shape[0]**2 + ground_truth_cropped.shape[1]**2))
        rotation_image = np.ones((diagonal, diagonal), np.uint8)*unknown_pixel_color
        ground_truth_rotated = self.rotate_image(ground_truth_cropped, best_angle_refined, create_rot_image= False,
                                                rot_image= rotation_image, border_value= unknown_pixel_color)
        _,ground_truth_thresh = cv2.threshold(ground_truth_rotated, 180, 255,cv2.THRESH_TOZERO)
        ground_truth_padded = self.addPadding(ground_truth_thresh, padded_cropped_map, centered= False, origin= best_coord,
                                            color = unknown_pixel_color)

        # Mask ground truth template with differences
        template_masked = cv2.bitwise_and(ground_truth_padded, ground_truth_padded,
                                    mask = combined_differences_mask)
        
        # Finally convert template_mask to BGR
        template_masked = cv2.cvtColor(template_masked, cv2.COLOR_GRAY2BGR, dstCn= 0)
        result = cv2.add(template_masked, combined_differences)

        # Compute number of pixels that represents occupied or unoccupied cell
        pixels_to_compare = cv2.countNonZero(rotated_template)
        wrong_pixels = cv2.countNonZero(~combined_differences_mask)

        percent_of_correct = 100 - float(wrong_pixels)/pixels_to_compare*100

        if percent_of_correct < 30:
            rospy.loginfo('Bad match or wrongly created map!')
            response = Map_comparsionResponse()
            response.successful = False
            return response
        
        else:

            response = Map_comparsionResponse()
            response.successful = True
            # Visualize the results
            self.visualizer.update(result, percent_of_correct)
            return response



    def getBorder(self, original, thresholded_wo_border, angle= None):
        # Make obstacle pixels 0
        _,original_inverted = cv2.threshold(original, 10, 255, cv2.THRESH_BINARY_INV)
        
        # Crop and rotate image if needed
        if angle != None:
            original_border,min_px,max_px = self.bbox2(original_inverted)
            original_border_rotated = self.rotate_image(original_border, angle)

        # Else crop both images and don't rotate
        else:
            thresholded_wo_border, min_px, max_px = self.bbox2(thresholded_wo_border)
            original_border_rotated = original_inverted[min_px[0]:max_px[0]+1, min_px[1]:max_px[1]+1]
        
        # Threshold after rotation - beacause of interpolation
        _,original_border_rotated_binary = cv2.threshold(original_border_rotated, 127, 255,cv2.THRESH_BINARY_INV)

        return original_border_rotated_binary, min_px, max_px

    def adjust_map(self, image):
        binary = self.mapToBinary(image)
        adjusted,_,_ = self.bbox2(binary)
        return adjusted


    def rotate_image(self, image, angle, create_rot_image= True, rot_image= None, border_value= None):
        diagonal = int(sqrt(image.shape[0]**2 + image.shape[1]**2))
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


    def bbox2(self,img):
        rows = np.any(img, axis=1)
        cols = np.any(img, axis=0)
        ymin, ymax = np.where(rows)[0][[0, -1]]
        xmin, xmax = np.where(cols)[0][[0, -1]]
        return img[ymin:ymax+1, xmin:xmax+1], (ymin,xmin), (ymax, xmax)

    def mapToBinary(self, img):
        img = ~cv2.inRange(img, 200, 210)
        # height = img.shape[0]
        # width = img.shape[1]
        # unknown_color_image = np.ones((height,width), np.uint8)*unknown_pixel_color
        # img = (unknown_color_image - img)
        return img

    def addPadding(self, to_pad, img_to_compare, centered = True, origin = None, color = 0):
        # Even if to_pad image is larger than template, padding has to be added because of sometimes
        # unavoidable padding on template
        if centered:
            height = int(max(to_pad.shape[0], img_to_compare.shape[0])*1.3)
            width = int(max(to_pad.shape[1], img_to_compare.shape[1])*1.3)
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

    def blendAndDrawDifference(self, img1, img2, alpha):
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

        return difference

class Visualization(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.image = None
        self.percentage = None
        self.daemon = True
        self.updated = Event()
        self.start()
    
    def update(self, image, percent_of_correct):
        self.updated.set()
        self.image = image
        self.percentage = percent_of_correct

    def run(self):
        while True:
            if self.updated.is_set():
                self.updated.clear()
                # Evaluate result 
                if self.percentage < 50:
                    evaluation = 'Try again!'
                elif self.percentage < 75:
                    evaluation = 'Well done!'
                else:
                    evaluation = 'Excellent!'

                font = cv2.FONT_HERSHEY_COMPLEX
                percentage = '%01.1f%% of map is correct.' % self.percentage

                # Visualize result
                cv2.putText(self.image,percentage,(20,30), font, 0.8,(0,0,0),1,cv2.LINE_AA)
                cv2.putText(self.image,evaluation,(20,55), font, 0.8,(0,0,0),1,cv2.LINE_AA)

                cv2.namedWindow('Comparsion',cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Comparsion', 1000,1000)
                cv2.imshow('Comparsion',self.image)

                # Press escape to close window
                if cv2.waitKey(0) == 27:
                    cv2.destroyAllWindows()
            else:
                rospy.sleep(0.1)

            


if __name__ == '__main__':
    try:
        map_comparator = Comparator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
