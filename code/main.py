#!/usr/bin/python
from twisted.internet import task
from twisted.internet import reactor
import time
import datetime
import os

from ImportDirectoryList import *
import L3G4200D as L3G
import LSM303DLM as LSM
import BMP085 as BMP
import GPS

class IMU_Params:
    def __init__(self, lsm, l3g):

def readIMU(arg):
    print 'IMU Read: ' + str(datetime.datetime.now())
    accel = arg[0]

def readBMP(arg):
    print 'BMP read: ' + str(datetime.datetime.now())

def readGPS(arg):
    print 'GPS read: ' + str(datetime.datetime.now())


if __name__ == '__main__':
    #Sensor Initializations
    l3g = L3G.L3G4200D()
    l3g.enableDefault()
    lsm = LSM.LSM303DLM()
    lsm.enableDefault()
    bmp = BMP.BMP085()
    gps = GPS.GPS()

    #Constants
    NSEW_limits = [46*100+10, 45*100+25, 72*100+20, 73*100+20]

    #Sampling Frequencies
    imu_fs = 10.0
    bmp_fs = 2.0
    gps_fs = 1.0

    #Create log files
    newpath = './log'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    imu_file = open(newpath+"/IMUData.txt","w")
    bmp_file = open(newpath+"/BMPData.txt","w")
    gps_file = open(newpath+"/GPSData.txt","w")

    imu_task = task.LoopingCall(readIMU,[l3g,lsm]).start(1.0/imu_fs)
    bmp_task = task.LoopingCall(readBMP,[bmp]).start(1.0/bmp_fs)
    gps_task = task.LoopingCall(readGPS,[gps]).start(1.0/gps_fs)

    reactor.run()

