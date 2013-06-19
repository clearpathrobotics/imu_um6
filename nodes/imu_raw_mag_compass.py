#!/usr/bin/env python
import roslib; roslib.load_manifest("imu_um6")
import rospy
import time
from geometry_msgs.msg import Vector3Stamped
from math import atan2

class ImuRawMagCompassNode:
    def __init__(self):
        rospy.init_node('imu_raw_mag_compass')
        rospy.Subscriber("mag", Vector3Stamped, self._cb)
        self.center = (81.319126072576225, -20.378599446924127, -321.76847826086959)
        rospy.spin()

    def _cb(self, data):
        data.vector.x -= self.center[0]
        data.vector.y -= self.center[1]
        #data.vector.z -= self.center[2]
        print -atan2(-data.vector.y, -data.vector.x)
        
if __name__ == '__main__':
    ImuRawMagCompassNode()

