#!/usr/bin/env python

import yaml


with open("/home/patrik/catkin_ws/src/mech_ros/map/muzeum_aruco_markers.yaml", 'r') as stream:
    data_loaded = yaml.load(stream)
    # frame_id  = data_loaded["frame_id"]
    landmarks = data_loaded["landmarks"]
    print(landmarks)

    for landmark in landmarks:
        # translation = landmark["Translation"]
        print(landmark)
    #     print(translation[0])
    # print(landmarks)
    # print(data_loaded["0"])
    list_r = landmarks[int("0")]["RPY"]
    print(list_r)