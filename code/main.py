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

def readIMU(arg):
    print 'IMU Read: ' + str(datetime.datetime.now())

def readBMP(arg):
    print 'BMP read: ' + str(datetime.datetime.now())

def readGPS(arg):
    print 'GPS read: ' + str(datetime.datetime.now())


if __name__ == '__main__':
    l3g = L3G.L3G4200D()
    l3g.enableDefault()
    lsm = LSM.LSM303DLM()
    lsm.enableDefault()
    bmp = BMP.BMP085()
    gps = GPS.GPS()

    #Create log files
    newpath = './log'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    imu_file = open(newpath+"/IMUData.txt","w")
    bmp_file = open(newpath+"/BMPData.txt","w")
    gps_file = open(newpath+"/GPSData.txt","w")

    imu_task = task.LoopingCall(readIMU,[l3g,lsm]).start(0.1)
    bmp_task = task.LoopingCall(readBMP,[bmp]).start(0.1)
    gps_task = task.LoopingCall(readGPS,[gps]).start(1.0)

    reactor.run()

