# Diploma thesis
---
> This repository contains ROS based software part of the project.
>
> The aim of the project is to create robot which can be operated manually or drive autonomously. Robot should be able to navigate to charging station, when it needs to, charge itself and procced to normal state completely without external supervision. Fiducials in environment are used for localization and for augmented reality.
---

## Description of robot:

### Sensors:
 - RPi camera
 - LIDAR - only for SLAM
 - IMU
 - 4 Wheel encoders

### Hardware:
 - Raspberry Pi 3B+
 - Custom PCB with Arduino SAMD Zero
 - Working station PC - Ubuntu

### Mechanics:
 - 4 Wheel Skid Steer
 - Front servo gripper
---
## Localization using fiducials:

Use of [ArUco](https://www.uco.es/investiga/grupos/ava/node/26 "ArUco") for localization in environment

<img src="/Markers/marker_1.png" height="300">

[logo]: https://raw.githubusercontent.com/Vpatrik/mech_ros/master/Markers/marker_1.png "ArUco marker"

---

## Augmented reality using fiducials:
> 
Hosted on another [repo](https://github.com/adyczech/AR_GUI "AR_GUI").

---

## Simulation - Created simulation environment in Gazebo, URDF robot model

### Working features in Gazebo:
 - [x] SLAM
 - [x] Localization using fiducials
 - [x] Navigation - use of <em>move_base_flex</em>
 - [x] State machine for navigation and part of charging cycle

## Real robot - Currently testing procedures in laboratory

### Working features in real world:
 - [x] SLAM
 - [x] Localization using fiducias
 - [x] Navigation - use of <em>move_base_flex</em>
 - [x] Following object with ArUco marker
 - [x] State machine for navigation and complete charging cycle
---

### State machine:

<em>Complete SMACH</em>:  


<img src="/SMACH/SM.svg" width="700">

<em>Navigation loop</em>:  

<img src="/SMACH/SM_navigation_loop.svg" width="400">

<em>Navigation to charging plug</em>:  

<img src="/SMACH/SM_navigate_2_charge.svg" width="600">

<em>Charging</em>:  

<img src="/SMACH/SM_charging.svg" height="400">

