#!/usr/bin/python
from twisted.internet import task
from twisted.internet import reactor
import time
import datetime
import os

import RPi.GPIO as GPIO

import L3G4200D as L3G
import LSM303DLM as LSM
import BMP085 as BMP

class IMU_Params:
    accel = []
    gyro = []
    mag = []

    def __init__(self, lsm, l3g):
        self.lsm = lsm
        self.l3g = l3g

class BMP_Params:
    alt = 0
    pressure = 0
    temp = 0

    def __init__(self, bmp):
        self.bmp = bmp

def readIMU(arg):
    print 'IMU Read: ' + str(datetime.datetime.now())
    arg[0].accel = arg[0].lsm.readRawAccel()
    arg[0].mag = arg[0].lsm.readRawMag()
    arg[0].gyro = arg[0].l3g.readRawGyro()
    print 'accel: ' + str(arg[0].accel) + ';gyro: ' + str(arg[0].gyro) + ';mag: ' + str(arg[0].mag)

def readBMP(arg):
    print 'BMP read: ' + str(datetime.datetime.now())
    arg[0].temp = arg[0].readTemperature()
    arg[0].pressure = arg[0].readPressure()
    arg[0].altitude = arg[0].readAltitude()
    print 'temp: ' + str(arg[0].temp) + ';pressure: ' + str(arg[0].pressure) + ';altitude: ' + str(arg[0].altitude)

def readGPS(arg):
    print 'GPS read: ' + str(datetime.datetime.now())

if __name__ == '__main__':
    #Create log files
    newpath = './log'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    imu_file = open(newpath+"/IMUData.txt","w")
    bmp_file = open(newpath+"/BMPData.txt","w")
    gps_file = open(newpath+"/GPSData.txt","w")

    #Sensor Initializations
    l3g = L3G.L3G4200D()
    l3g.enableDefault()
    lsm = LSM.LSM303DLM()
    lsm.enableDefault()

    gps = GPS.GPS()

    #Constants
    NSEW_limits = [46*100+10, 45*100+25, 72*100+20, 73*100+20]
    start_time = time.time()*1000.0

    #Sampling Frequencies
    imu_fs = 10.0
    bmp_fs = 2.0
    gps_fs = 1.0

    #Object container initializations
    imu = IMU_Params(lsm, l3g)
    bmp = BMP_Params(BMP.BMP085())

    imu_task = task.LoopingCall(readIMU,[imu]).start(1.0/imu_fs)
    bmp_task = task.LoopingCall(readBMP,[bmp]).start(1.0/bmp_fs)
    gps_task = task.LoopingCall(readGPS,[gps]).start(1.0/gps_fs)

    reactor.run()

