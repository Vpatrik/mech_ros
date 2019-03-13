#!/usr/bin/env python

# Patrik Vavra 2019

import rospy

from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from std_srvs.srv import TriggerResponse

from std_msgs.msg import Bool

class ChargingStation:
    def __init__(self):
        rospy.init_node('RPi_docking_station')
        
        self.solenoid_lock = True
        self.endstop = Bool()

        self.solenoid_service = rospy.Service('/set_solenoid', Trigger, self.set_solenoid)
        self.endstop_publisher = rospy.Publisher('/endstop', Bool, queue_size=5)

    def set_solenoid(self, req):

        # Tady by mel byt prikaz pro uvolneni solenoidu pres I2C

        response = TriggerResponse()
        rospy.loginfo("Solenoid unlocked!")
        response.success = True
        response.message = "Unlocked"
       
        return response

    def run(self):
        rate = rospy.Rate(10) # Rate of loop in [Hz]
        while not rospy.is_shutdown():
            
            # Tady by melo vycitani stavu koncaku
            self.endstop.data = False

            self.endstop_publisher.publish(self.endstop)
            rate.sleep()

    
if __name__ == "__main__":
    station = ChargingStation()
    try:
        station.run()
    except rospy.ROSInterruptException:
        pass
