import numpy as np
import math
import matplotlib.pyplot as plt

filename1 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Error_X_Yaw.txt"
filename2 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Error_Y_Yaw.txt"
filename3 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Error_Yaw_mark_Yaw.txt"


filename4 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Surf_x.txt"
filename5 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Surf_y.txt"
filename6 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/Surf_yaw.txt"

# filename1 = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/rqt_multiplot_camera_compl_y_Yaw_dependency.txt"
# filename = "/home/patrik/catkin_ws/src/mech_ros/src/Mereni_potom_zkopirovat/rqt_multiplot_camera_compl_xy_dependency.txt"


# Popisky grafu
title = "Pose estimation dependent on marker Yaw"
x_str = "Yaw board[deg]"
y_str3 = "Yaw marker error [deg]"
y_str1 = "X marker error [m]"
y_str2 = "Y marker error [m]"

x_str4 = "Surface [pixel*pixel]"
y_str6 = "Yaw marker error [deg]"
y_str4 = "X marker error [m]"
y_str5 = "Y marker error [m]"

title2 = "Pose estimation dependent on Marker Surface"

def computeCov(m):
    cov = np.cov(m,y = None, rowvar=False)
    corr = np.corrcoef(m,y=None, rowvar=False)
    print(cov)
    print(corr)

def myCov(x,y):
    cov = 0
    cov_x = 0
    cov_y = 0
    for i in range(len(x)):
        cov+=(-x[i])*(-y[i])
        cov_x += x[i]**2
        cov_y += y[i]**2
    cov_xy = cov/(len(x)+1)
    cov_x = cov_x/(len(x)+1)
    cov_y = cov_y/(len(x)+1)
    corr_xy = cov_xy/(cov_x**0.5*cov_y**0.5)
    return cov_x, cov_y, cov_xy, corr_xy

def parse(filename):
    with open(filename, "r") as file_a:
        lines = file_a.readlines()
    string = [s.split(",") for s  in lines[2:] ]
    x = np.array([float(d[0]) for d in string])
    y = np.array([float(d[1]) for d in string])

    return x,y

def recalculateAngle(x):
# Recalculate for angle
    for i in range(len(x)):
        if x[i] > math.pi/2:
            x[i]-=math.pi
        if x[i] < -math.pi/2:
            x[i]+=math.pi
    x = x*180/math.pi
    return x


x,y1 = parse(filename1)
_,y2 = parse(filename2)
_,y3 = parse(filename3)

x = recalculateAngle(x)


x4,y4 = parse(filename4)
_,y5 = parse(filename5)
_,y6 = parse(filename6)

m = np.stack((x,y1,y2,y3), axis =-1)


# computeCov(m)
cov_xy = myCov(y4,y6)
print(cov_xy)

# Plot data
# First figure
plt.figure(1, figsize=(15, 10))

plt.subplot(3,1,3)
lines = plt.plot(x,y3)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
plt.axis([-60, 50, -1.5, 1.5])
plt.grid(True)
plt.xlabel(x_str)
plt.ylabel(y_str3)


plt.subplot(3,1,2)
lines = plt.plot(x,y2)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
plt.axis([-60, 50, -0.005, 0.005])
plt.grid(True)
plt.ylabel(y_str2)


plt.subplot(3,1,1)
lines = plt.plot(x,y1)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
plt.axis([-60, 50, -0.02, 0.02])
plt.grid(True)
plt.ylabel(y_str1)

plt.title(title)


# Second figure
plt.figure(2, figsize=(15, 10))

plt.subplot(3,1,1)
lines = plt.plot(x4,y4)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
# plt.axis([-60, 50, -0.005, 0.005])
plt.grid(True)
plt.ylabel(y_str1)
plt.title(title2)

plt.subplot(3,1,2)
lines = plt.plot(x4,y5)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
# plt.axis([-60, 50, -0.005, 0.005])
plt.grid(True)
plt.ylabel(y_str2)

plt.subplot(3,1,3)
lines = plt.plot(x4,y6)
plt.setp(lines, color = "r", marker = ".", markersize = 5, linestyle = "")
# plt.axis([-60, 50, -0.005, 0.005])
plt.grid(True)
plt.ylabel(y_str3)
plt.xlabel(x_str4)


plt.figure(3)
# evenly sampled time at 200ms intervals
t = np.arange(1000., 15500., 10)

# start = math.pi/2
# stop = 3*math.pi/2
# t = np.arange(start, stop, 0.05)
# cos = abs(np.cos(t))
# sin = abs(np.sin(t))


line1, = plt.plot(t, 2248*t**-1.6705, 'r',label='Manual measurement', linewidth=2)
line2, = plt.plot( t, 14148752282*t**-3.74312, 'b',label='Difference measurement', linewidth=2)
# line1, = plt.plot(t*180/math.pi, cos, 'r',label='Manual measurement', linewidth=3)
# line2, = plt.plot( t*180/math.pi, sin, 'b',label='Difference measurement', linewidth=3)
plt.legend([line1, line2],['Manual measurement','Difference measurement' ])


plt.show()

