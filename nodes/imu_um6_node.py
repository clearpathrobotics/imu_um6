#! /usr/bin/env python
import roslib; roslib.load_manifest('imu_um6')

import cProfile
import rospy
import math
import numpy
import tf
from time import sleep

from um6.driver import Um6Drv
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3Stamped
from imu_um6.srv import *
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis

class ImuUm6Node(object):

    first = True

    def __init__(self, default_port='/dev/ttyUSB0'):
        """
        @param default_port: default serial port to use for
            establishing a connection to the UM6 IMU sensor.
            This will be overridden by ~port param if 
            available.
        """
        rospy.init_node('imu_um6')

        self.port = rospy.get_param('~port', default_port)
        self.frame_id = rospy.get_param('~frame_id', "/imu")
        self.throttle_rate = rospy.get_param('~throttle_rate', 10)
        self.reset_mag = rospy.get_param('~reset_mag', True)
        self.reset_accel = rospy.get_param('~reset_accel', True)
        self.mag_zero_x = rospy.get_param('~mag_zero_x', 0.0)
        self.mag_zero_y = rospy.get_param('~mag_zero_y', 0.0)
        self.mag_zero_z = rospy.get_param('~mag_zero_z', 0.0)
        rospy.loginfo("serial port: %s"%(self.port))
        rospy.loginfo("Magnetometer calibration: %.3f %.3f %.3f",self.mag_zero_x,self.mag_zero_y,self.mag_zero_z)

        self.imu_data = Imu()
        self.imu_data = Imu(header=rospy.Header(frame_id="imu_link"))

        self.imu_data.orientation_covariance = [1e6, 0, 0, 
                                                0, 1e6, 0, 
                                                0, 0, 1e-6]

        self.imu_data.angular_velocity_covariance = [1e6, 0, 0,
                                                     0, 1e6, 0, 
                                                     0, 0, 1e-6]

        self.imu_data.linear_acceleration_covariance = [1e6, 0, 0, 
                                                        0, 1e6, 0, 
                                                        0, 0, 1e-6]

        self.imu_pub = rospy.Publisher('imu/data', Imu)

        self.rpy_data = Vector3Stamped()
        self.rpy_pub = rospy.Publisher('imu/rpy', Vector3Stamped)

        self.mag_data = Vector3Stamped()
        self.mag_pub = rospy.Publisher('imu/mag', Vector3Stamped)

        # what data to pass to callback
        dataMask = (Um6Drv.DATA_QUATERNION | 
                    Um6Drv.DATA_ROLL_PITCH_YAW | 
                    Um6Drv.DATA_LINEAR_ACCEL | 
                    Um6Drv.DATA_ANGULAR_VEL | Um6Drv.DATA_MAGNETOMETER)

        self.driver = Um6Drv(self.port, dataMask, self.um6_data_cb)
        
        self.received = -1
        cmd_seq = [Um6Drv.CMD_ZERO_GYROS, Um6Drv.CMD_RESET_EKF]
        if self.reset_mag:
            cmd_seq.append(Um6Drv.CMD_SET_MAG_REF)
        if self.reset_accel:
            cmd_seq.append(Um6Drv.CMD_SET_ACCEL_REF)
        cmd_seq += [(Um6Drv.UM6_MISC,Um6Drv.UM6_MISC_DATA),
                (Um6Drv.UM6_COMMUNICATION,Um6Drv.UM6_COMMUNICATION_DATA)]
        while (not rospy.is_shutdown()) and (len(cmd_seq)>0):
            cmd = cmd_seq[0]
            if type(cmd)==tuple:
                (cmd,data) = cmd
                self.driver.sendConfig(cmd, data, self.um6_cmd_cb)
            else:
                self.driver.sendCommand(cmd, self.um6_cmd_cb);
            start = rospy.Time.now()
            while (rospy.Time.now() - start).to_sec() < 0.5:
                self.driver.updateBlocking(0.5)
                if self.received == cmd:
                    break
            if self.received == cmd:
                self.received = -1
                cmd_seq = cmd_seq[1:]
        rospy.loginfo("Imu initialisation completed")

        self.reset_srv = rospy.Service('imu/reset', Reset, self.reset_service_cb)

        # Send a first packet to reset the communication
        # zero the gyros, reset the kalman filter, reset reference headings
        # self.driver.sendCommand(Um6Drv.CMD_ZERO_GYROS, self.um6_cmd_cb);sleep(0.5)
        # self.driver.sendCommand(Um6Drv.CMD_RESET_EKF, self.um6_cmd_cb);sleep(0.5)
        # self.driver.sendCommand(Um6Drv.CMD_SET_MAG_REF, self.um6_cmd_cb);sleep(0.5)
        # self.driver.sendCommand(Um6Drv.CMD_SET_ACCEL_REF, self.um6_cmd_cb);sleep(0.5)
        # self.driver.sendConfig(Um6Drv.UM6_MISC, Um6Drv.UM6_MISC_DATA, self.um6_cmd_cb)
        # self.driver.sendConfig(Um6Drv.UM6_COMMUNICATION, Um6Drv.UM6_COMMUNICATION_DATA, self.um6_cmd_cb)

        while not rospy.is_shutdown():
            self.driver.updateBlocking()

    def um6_cmd_cb(self, cmd, result):
        if (cmd == Um6Drv.UM6_COMMUNICATION):
            rospy.loginfo("Set quaternion output: %s"%(result))
        if (cmd == Um6Drv.UM6_MISC):
            rospy.loginfo("Configured EKF: %s"%(result))
        if (cmd == Um6Drv.CMD_RESET_EKF):
            rospy.loginfo("Reset EKF: %s"%(result))
        if (cmd == Um6Drv.CMD_ZERO_GYROS):
            rospy.loginfo("Zero Gyros: %s"%(result))
        if (cmd == Um6Drv.CMD_SET_MAG_REF):
            rospy.loginfo("Set Magnetometer Reference: %s"%(result))
        if (cmd == Um6Drv.CMD_SET_ACCEL_REF):
            rospy.loginfo("Set Accelerometer Reference: %s"%(result))
        if result:
            self.received = cmd

    def reset_service_cb(self,req):
        cmd_seq = []
        if req.zero_gyros:
            cmd_seq.append(Um6Drv.CMD_ZERO_GYROS)
        if req.reset_ekf:
            cmd_seq.append(Um6Drv.CMD_RESET_EKF)
        if req.set_mag_ref:
            cmd_seq.append(Um6Drv.CMD_SET_MAG_REF)
        if req.set_accel_ref:
            cmd_seq.append(Um6Drv.CMD_SET_ACCEL_REF)
        self.received = -1
        while (not rospy.is_shutdown()) and (len(cmd_seq)>0):
            cmd = cmd_seq[0]
            self.driver.sendCommand(cmd, self.um6_cmd_cb);
            start = rospy.Time.now()
            while (rospy.Time.now() - start).to_sec() < 0.5:
                rospy.sleep(0.01)
                if self.received == cmd:
                    break
            if self.received == cmd:
                self.received = -1
                cmd_seq = cmd_seq[1:]
        rospy.loginfo("Imu initialisation completed")
        return ResetResponse()


    def um6_data_cb(self, data):
        now = rospy.Time.now()
        if (now.to_sec() - self.imu_data.header.stamp.to_sec())*self.throttle_rate < 1.:
            # Ignore data at this rate (ok for a boat)
            return
        self.imu_data.header.frame_id = self.frame_id
        self.imu_data.header.stamp = now
        self.imu_data.orientation = Quaternion()
        # print data

        # IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
        q = [data['DATA_QUATERNION'][2],
             data['DATA_QUATERNION'][1],
            -data['DATA_QUATERNION'][3],
             data['DATA_QUATERNION'][0]]

        self.imu_data.orientation.x = q[0] 
        self.imu_data.orientation.y = q[1]
        self.imu_data.orientation.z = q[2]
        self.imu_data.orientation.w = q[3] 

        # convert to radians from degrees
        # again note NED to ENU converstion
        self.imu_data.angular_velocity.x = data['DATA_ANGULAR_VEL'][1] * (math.pi/180.0)
        self.imu_data.angular_velocity.y = data['DATA_ANGULAR_VEL'][0] * (math.pi/180.0)
        self.imu_data.angular_velocity.z = -(data['DATA_ANGULAR_VEL'][2] * (math.pi/180.0))
        # again note NED to ENU converstion
        self.imu_data.linear_acceleration.x = data['DATA_LINEAR_ACCEL'][1]
        self.imu_data.linear_acceleration.y = data['DATA_LINEAR_ACCEL'][0]
        self.imu_data.linear_acceleration.z = -(data['DATA_LINEAR_ACCEL'][2])
        
        self.imu_pub.publish(self.imu_data)

        self.mag_data.header = self.imu_data.header
        self.mag_data.vector.x = data['DATA_MAGNETOMETER'][0]-self.mag_zero_x
        self.mag_data.vector.y = data['DATA_MAGNETOMETER'][1]-self.mag_zero_y
        self.mag_data.vector.z = data['DATA_MAGNETOMETER'][2]-self.mag_zero_z
        self.mag_pub.publish(self.mag_data)

        self.rpy_data.header = self.imu_data.header
        self.rpy_data.vector.x = -data['DATA_ROLL_PITCH_YAW'][0] * (math.pi/180.0)
        self.rpy_data.vector.y = -data['DATA_ROLL_PITCH_YAW'][1] * (math.pi/180.0)
        self.rpy_data.vector.z = data['DATA_ROLL_PITCH_YAW'][2] * (math.pi/180.0)
        # self.rpy_data.vector.z = -math.pi/2 - math.atan2(self.mag_data.vector.y,self.mag_data.vector.x) # ENU
        self.rpy_pub.publish(self.rpy_data)

if __name__ == '__main__':
    node = ImuUm6Node()
    # cProfile.run("node = ImuUm6Node()")
