#!/usr/bin/env python
import serial
import struct
import math
from select import select
from time import sleep

"""
    Simple serial (RS232) driver for the CH Robotics UM6 IMU

    NOTE: this driver expects a number of things to be true:
    1. Broadcast mode is enabled
    2. QUATERNION output is enabled
    2. EULER output is enabled
    2. ANGULAR VELOCITY output is enabled
    2. LINEAR ACCEL output is enabled

    Use CHRobotics Serial Interface program 
    to set the appropriate outputs

    see main() example at end of file for usage
            
"""
class Um6Drv:

    # data ouput mask values
    DATA_QUATERNION       = 0x01
    DATA_ROLL_PITCH_YAW   = 0x02
    DATA_ANGULAR_VEL      = 0x04
    DATA_LINEAR_ACCEL     = 0x08
    DATA_MAGNETOMETER     = 0x10

    # configuration registers
    UM6_COMMUNICATION   = 0x00
    # configuration data
    # [0-7] broadcast_rate: 0 (20 Hz)
    # [8-10] baud rate: 5 (115200)
    # [11-13] GPS_baud_rate: 6 (N/A)
    # [14] RESERVED
    # [15] Satellite status
    # [16] Satellite summary
    # [17] GPS velocity 
    # [18] GPS relative position
    # [19] GPS position
    # [20] Temperature
    # [21] Covariance
    # [22] Euler angle: 1
    # [23] Quaternion: 1
    # [24] Proc. magnetometer: 1
    # [25] Proc. accelerometer: 1
    # [26] Proc. gyro: 1
    # [27] Raw magnetometer
    # [28] Raw accelerometer
    # [29] Raw gyro 
    # [30] Broadcast mode: 1
    # [31] RESERVED
    # Default value, do funky stuff with the magnetometer data
    # UM6_COMMUNICATION_DATA = 0x47c00500
    # Magnetometer is RAW
    UM6_COMMUNICATION_DATA = 0x4Ec00500

    # configuration registers
    UM6_MISC   = 0x01
    # configuration data
    # [0-26] reserved
    # [27] PPS enabled
    # [28] Quaternion state estimation: 1
    # [29] Auto calibration of gyros
    # [30] EKF uses accelerometer
    # [31] EKF uses magnetometer
    UM6_MISC_DATA = 0x78000000

    # data registers
    UM6_RAW_MAG_PROC_XY      = 0x5A
    UM6_RAW_MAG_PROC_Z       = 0x5B
    UM6_GYRO_PROC_XY     = 0x5C
    UM6_GYRO_PROC_Z      = 0x5D
    UM6_ACCEL_PROC_XY    = 0x5E
    UM6_ACCEL_PROC_Z     = 0x5F
    UM6_MAG_PROC_XY      = 0x60
    UM6_MAG_PROC_Z       = 0x61
    UM6_EULER_PHI_THETA  = 0x62
    UM6_EULER_PSI        = 0x63
    UM6_QUAT_AB          = 0x64
    UM6_QUAT_CD          = 0x65

    # command registers
    CMD_ZERO_GYROS       = 0xAC
    CMD_RESET_EKF        = 0xAD
    CMD_SET_ACCEL_REF    = 0xAF
    CMD_SET_MAG_REF      = 0xB0
    CMD_BAD_CHECKSUM     = 0xFD # sent by UM6 when it receives corrupted pkt

    # set default command callbacks to 'None'
    cmd_cb = { UM6_COMMUNICATION: None,
               CMD_ZERO_GYROS: None,
               CMD_RESET_EKF: None,
               CMD_SET_ACCEL_REF: None,
               CMD_SET_MAG_REF: None,
               CMD_BAD_CHECKSUM: None }

    #index values for packet arrays
    PKT_TYPE_IDX = 0
    PKT_ADDR_IDX = 1
    PKT_DATA_IDX = 2

    # scale factors for register output
    SCALE_QUAT  = 0.0000335693
    SCALE_EULER = 0.0109863
    SCALE_GYRO  = 0.0610352
    SCALE_MAG  =  0.000305176
    SCALE_ACCEL = 0.000183105

    NO_DATA_PACKET       = 0x00

    quaternion = []
    wip_quat = [] # tmp var for building quats from register reads

    valid = {'quaternion': False,
               'mag_x': False,
               'mag_y': False,
               'mag_z': False,
               'lin_acc_x': False,
               'lin_acc_y': False,
               'lin_acc_z': False,
               'ang_vel_x': False,
               'ang_vel_y': False,
               'ang_vel_z': False,
               'yaw': False,
               'roll': False,
               'pitch': False }

    def __init__(self, port, outputDataMask, cb=None):
        self.ser = serial.Serial(port, 
                                    baudrate=115200, 
                                    bytesize=8, parity='N', 
                                    stopbits=1, timeout=None)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.buf = ""
        self.data_callback = cb
        self.output = outputDataMask

    def __del__(self):
        self.ser.close()

    def update(self):
        pkt = self.readPacket()
        if (pkt != Um6Drv.NO_DATA_PACKET):       
            self.decodePacket(pkt)
        else:
            sleep(0.001)

    def waitData(self, nbytes=1, timeout=1.0):
        while True:
            self.buf += self.ser.read(self.ser.inWaiting())
            if len(self.buf)>=nbytes:
                return True
            rlist, _, _ = select([ self.ser ], [], [], timeout)
            if rlist<=0:
                return False
            # Now sleep for the expected time necessary to receive these bits
            sleep(1e-4*(nbytes-len(self.buf)))

    def updateBlocking(self, timeout=1.0):
        rlist, _, _ = select([ self.ser ], [], [], timeout)
        if rlist:
            self.update()

    def syncToHeader(self):
        if self.waitData(3):
            idx = self.buf.find("snp")
            if idx>=0:
                self.buf = self.buf[idx+3:]
                return True
        return False 

    def sendConfig(self, reg, data, callback):
        """ Sends a 4 byte configuration message """
        self.cmd_cb[reg] = callback
        pkt = [0x80,reg]
        for i in [3,2,1,0]:
            # Splits input into bytes
            pkt.append((data & (0xFF << 8*i)) >> 8*i)
        self.writePacket(pkt)

    def sendCommand(self, cmd, callback):
        self.cmd_cb[cmd] = callback
        pkt = [0, cmd]
        self.writePacket(pkt)

    def writePacket(self, pkt):

        #header
        self.ser.write('s')
        self.ser.write('n')
        self.ser.write('p')

        chkSum = (self.chToByte("s") +
                  self.chToByte("n") +
                  self.chToByte("p"))

        for i in range(0, len(pkt)):
            self.ser.write(chr(pkt[i]))
            chkSum += pkt[i]

        strChkSum = ("%s"%(chkSum))

        high = (chkSum >> 8) & 0x00FF
        low = 0x00FF & chkSum

        self.ser.write(chr(high))
        self.ser.write(chr(low))

    def readPacket(self):
        if (not self.syncToHeader()):
            return Um6Drv.NO_DATA_PACKET
        if not self.waitData(2):
            return Um6Drv.NO_DATA_PACKET

        packetType = self.chToByte(self.buf[0])

        hasData = packetType >> 7
        isBatch = (packetType >> 6) & 0x01
        batchLen = (packetType >> 2) & 0x0F

        #print "hasData=%s, isBatch=%s, batchLen=%s" % (hasData,isBatch,batchLen)
        addr = self.chToByte(self.buf[1])

        calcChkSum = (self.chToByte("s") +
                      self.chToByte("n") +
                      self.chToByte("p") +
                      packetType + addr)

        packet = []
        packet.append(packetType)
        packet.append(addr)

        self.buf = self.buf[2:]
        if not self.waitData(batchLen*4+2):
            return Um6Drv.NO_DATA_PACKET

        for i in range(0, batchLen * 4): # 4 bytes per register
            byte = self.chToByte(self.buf[i])
            packet.append(byte)
            calcChkSum += byte
        self.buf = self.buf[batchLen*4:]

        high = self.chToByte(self.buf[0])
        low  = self.chToByte(self.buf[1]) 

        chkSum = self.bytesToShort(high,low)
        self.buf = self.buf[2:]

        if (calcChkSum != chkSum):
            print "Read Pkt: BAD CHECKSUM"

        return packet

    def decodePacket(self, pkt):
        pt = pkt[Um6Drv.PKT_TYPE_IDX] 
        batchLen = (pt >> 2) & 0x0F
        startAddr = pkt[Um6Drv.PKT_ADDR_IDX]
        dataIdx = Um6Drv.PKT_DATA_IDX

        addr = startAddr

        result = 0 == (pt & 0x01)

        try:
            if (self.cmd_cb[addr] is not None):
                self.cmd_cb[addr](addr, result)
        except KeyError:
            self.cmd_cb[addr] = None

        if (addr == Um6Drv.CMD_BAD_CHECKSUM):
            print "Rx Packet: BAD CHECKSUM"

        for i in range(0, batchLen):
            self.parseAndSendData(addr+i, pkt[dataIdx+(i*4):dataIdx+((i+1)*4)])

    def parseAndSendData(self, addr, data):
        if (addr == Um6Drv.UM6_GYRO_PROC_XY):
            self.ang_vel_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_GYRO)
            self.ang_vel_y = ((self.bytesToShort(data[2],data[3])) * 
                                Um6Drv.SCALE_GYRO)
            self.valid['ang_vel_x'] = True 
            self.valid['ang_vel_y'] = True 

        if (addr == Um6Drv.UM6_GYRO_PROC_Z):
            self.ang_vel_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_GYRO)
            self.valid['ang_vel_z'] = True 

        if (addr == Um6Drv.UM6_RAW_MAG_PROC_XY):
            self.mag_x = (self.bytesToShort(data[0],data[1]))
            self.mag_y = (self.bytesToShort(data[2],data[3]))
            self.valid['mag_x'] = True 
            self.valid['mag_y'] = True 

        if (addr == Um6Drv.UM6_RAW_MAG_PROC_Z):
            self.mag_z = (self.bytesToShort(data[0],data[1])) 
            self.valid['mag_z'] = True 

        if (addr == Um6Drv.UM6_MAG_PROC_XY):
            self.mag_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_MAG)
            self.mag_y = ((self.bytesToShort(data[2],data[3])) * 
                                Um6Drv.SCALE_MAG)
            self.valid['mag_x'] = True 
            self.valid['mag_y'] = True 

        if (addr == Um6Drv.UM6_MAG_PROC_Z):
            self.mag_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_MAG)
            self.valid['mag_z'] = True 

        if (addr == Um6Drv.UM6_ACCEL_PROC_XY):
            self.lin_acc_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_ACCEL)
            self.lin_acc_y = ((self.bytesToShort(data[2],data[3])) *
                                Um6Drv.SCALE_ACCEL)
            self.valid['lin_acc_x'] = True 
            self.valid['lin_acc_y'] = True 

        if (addr == Um6Drv.UM6_ACCEL_PROC_Z):
            self.lin_acc_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_ACCEL)
            self.valid['lin_acc_z'] = True 

        if (addr == Um6Drv.UM6_QUAT_AB):
            a = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_QUAT
            b = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_QUAT
            self.wip_quat.append(a)
            self.wip_quat.append(b)

        if (addr == Um6Drv.UM6_EULER_PHI_THETA):
            self.roll = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_EULER
            self.pitch = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_EULER
            self.valid['roll'] = True
            self.valid['pitch'] = True

        if (addr == Um6Drv.UM6_EULER_PSI):
            self.yaw = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_EULER
            self.valid['yaw'] = True

        if (addr == Um6Drv.UM6_QUAT_CD):
            c = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_QUAT
            d = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_QUAT
            self.wip_quat.append(c)
            self.wip_quat.append(d)
            self.quaternion = []
            self.quaternion.append(self.wip_quat[0])
            self.quaternion.append(self.wip_quat[1])
            self.quaternion.append(self.wip_quat[2])
            self.quaternion.append(self.wip_quat[3])
            self.wip_quat = []
            self.valid['quaternion'] = True 

        if (self.valid['quaternion'] == True and
                self.valid['ang_vel_x'] == True and
                self.valid['ang_vel_y'] == True and
                self.valid['ang_vel_z'] == True and
                self.valid['lin_acc_x'] == True and
                self.valid['lin_acc_y'] == True and
                self.valid['lin_acc_z'] == True and
                self.valid['pitch'] == True and
                self.valid['roll'] == True and
                self.valid['yaw'] == True):

            self.valid['quaternion'] = False
            self.valid['ang_vel_x'] = False
            self.valid['ang_vel_y'] = False
            self.valid['ang_vel_z'] = False
            self.valid['lin_acc_x'] = False
            self.valid['lin_acc_y'] = False
            self.valid['lin_acc_z'] = False
            self.valid['pitch'] = False
            self.valid['roll'] = False
            self.valid['yaw'] = False

            results = {}
            if ((self.output & Um6Drv.DATA_QUATERNION) != 0):
                results['DATA_QUATERNION'] = self.quaternion
            if ((self.output & Um6Drv.DATA_ANGULAR_VEL) != 0):
                results['DATA_ANGULAR_VEL'] = [self.ang_vel_x, 
                                               self.ang_vel_y, 
                                               self.ang_vel_z]
            if ((self.output & Um6Drv.DATA_MAGNETOMETER) != 0):
                results['DATA_MAGNETOMETER'] = [self.mag_x, 
                                               self.mag_y, 
                                               self.mag_z]
            if ((self.output & Um6Drv.DATA_LINEAR_ACCEL) != 0):
                results['DATA_LINEAR_ACCEL'] = [self.lin_acc_x, 
                                                self.lin_acc_y, 
                                                self.lin_acc_z]
            if ((self.output & Um6Drv.DATA_ROLL_PITCH_YAW) != 0):
                results['DATA_ROLL_PITCH_YAW'] = [self.roll, 
                                                  self.pitch, 
                                                  self.yaw]
            if (self.data_callback is not None):
                self.data_callback(results)

    def bytesToShort(self, high, low):
        # convert string of low and high bytes to signed short
        return struct.unpack("h", chr(low) + chr(high))[0]

    def chToByte(self, ch):
        # convert single char string to unsigned byte
        return struct.unpack("B", ch)[0]

if __name__ == '__main__':
    ### EXAMPLE USAGE ###

    # callbacks (data and command)
    def data_cb(data): print "rx pkt"
    def cmd_cb(cmd, result): print "CMD %s Success: %s"%(cmd,result)

    # what to output in the data callback (quat & RPY in this example)
    dataMask = Um6Drv.DATA_QUATERNION | Um6Drv.DATA_ROLL_PITCH_YAW

    # init
    um6 = Um6Drv(3, dataMask, data_cb) #3 = COM4 in windows, use '/dev/ttyUSB' style in unix

    # send some commands
    um6.sendConfig(Um6Drv.UM6_COMMUNICATION, Um6Drv.UM6_COMMUNICATION_DATA, cmd_cb)
    um6.sendCommand(Um6Drv.CMD_ZERO_GYROS, cmd_cb)
    um6.sendCommand(Um6Drv.CMD_RESET_EKF, cmd_cb)
    um6.sendCommand(Um6Drv.CMD_SET_MAG_REF, cmd_cb)
    um6.sendCommand(Um6Drv.CMD_SET_ACCEL_REF, cmd_cb)

    # update UM6 state
    while(True):
        um6.update()
