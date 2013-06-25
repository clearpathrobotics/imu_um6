#!/usr/bin/env python
import roslib; roslib.load_manifest("imu_um6")
import rospy
import time
from geometry_msgs.msg import Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from math import atan2
#from tf.transformations import euler_to_quaternion
import tf

class ImuRawMagCompassNode:
    def __init__(self):
        rospy.init_node('imu_raw_mag_compass')

        self.mag_zero_x = rospy.get_param('~mag_zero_x')
        self.mag_zero_y = rospy.get_param('~mag_zero_y')
        self.mag_zero_z = rospy.get_param('~mag_zero_z')
        #self.mag_zero_radius = rospy.get_param('~mag_zero_radius')
        self.max_age = rospy.Duration.from_sec(rospy.get_param('~max_age_seconds', 0.11))

        self.imu_pub = rospy.Publisher("imu/data_compass", Imu)
        self.compass_pub = rospy.Publisher("imu/compass", Vector3Stamped)
        self.compass_msg = Vector3Stamped()

        rospy.Subscriber("imu/mag", Vector3Stamped, self._mag_cb)
        rospy.Subscriber("imu/data", Imu, self._imu_cb)

    def _mag_cb(self, data):
        data.vector.x -= self.mag_zero_x
        data.vector.y -= self.mag_zero_y
        data.vector.z -= self.mag_zero_z

        # Fixed for now. Later may determine up-vector from accelerometer.
        self.compass_msg.vector.z = atan2(-data.vector.x, data.vector.y)

        self.compass_msg.header.stamp = data.header.stamp
        self.compass_pub.publish(self.compass_msg)

    def _imu_cb(self, data):
        age = data.header.stamp - self.compass_msg.header.stamp
        if age > self.max_age:
            return

        orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.compass_msg.vector.z));

        # For now just overwrite wholesale, eliminating roll and pitch.
        data.orientation = orient

        self.imu_pub.publish(data)
        
    def spin(self):
        rospy.spin()

        
if __name__ == '__main__':
    ImuRawMagCompassNode().spin()

