#!/usr/bin/env python

import rospy
import sys


from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion
import math




# Define rotation vector
declination = 20*math.pi/180
rot_y_north = 0 # Rotation bewtween map y and geographic north
q_rot = quaternion_from_euler(0,0,-declination-rot_y_north)
# q_rot = Quaternion(*q_rot)

# Define transformation frames
frame_id = "mechROS/tf/odom"
child_frame_id = "/mechROS_base_link"


# Define topics
publish_imu_topic = "imu/data_corrected"
imu_magdwick_topic = "/imu/data"



def imu_madgwick_callback(imu_m):
    q_corrected = quaternion_multiply(q_rot,[imu_m.orientation.x,imu_m.orientation.y,imu_m.orientation.z,imu_m.orientation.w])
    # q_corrected.normalize()
    imu_m.orientation.x = q_corrected[0]
    imu_m.orientation.y = q_corrected[1]
    imu_m.orientation.z = q_corrected[2]
    imu_m.orientation.w = q_corrected[3]

     # Publish corrected topic
    imu_publisher.publish(imu_m)





if __name__ == '__main__':
    rospy.init_node('Declination_corrector', anonymous=True)

    imu_magdwick_subscriber = rospy.Subscriber(imu_magdwick_topic, Imu, imu_madgwick_callback)
    imu_publisher = rospy.Publisher(publish_imu_topic, Imu,queue_size=10)

    rospy.spin()
