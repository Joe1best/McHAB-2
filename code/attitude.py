#!/usr/bin/python

import serial
import struct
import numpy as np
import os
import math
import _transformations as tf
import sys
import time

import LSM303DLM as LSM
import L3G4200D as L3G

class attitude:
    F_sample = 20
    T_sample = 1.0/F_sample
    GYRO_FS = 250 #16 bit register

    Cbi_hat = np.identity(3)
    g_i = np.array([[0],[0],[-1]])
    b_d = np.array([[0], [0], [0]])
    C_d = np.identity(3)

    C_em = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    initial=time.time()*1000.0

    def __init__(self):
        self.firstRun = True

    def normalize(self,array):
        temp=math.sqrt(array[0][0]**2+array[1][0]**2+array[2][0]**2)
        return np.array([[array[0][0]/temp],[array[1][0]/temp],[array[2][0]/temp]])

    def convert_gyro(self,array):
        conv = lambda x: (float(x)/(2**15))*self.GYRO_FS*math.pi/180.0
        return np.array([[conv(array[0][0])],[conv(array[1][0])],[conv(array[2][0])]])

    def cross(self,array):
        return np.array([[0, -array[2][0], array[1][0]],[array[2][0],0,-array[0][0]],[-array[1][0],array[0][0],0]])

    def parseData(self,line_u):
        raw_accel=np.array([[line_u[0]],[line_u[1]],[line_u[2]]])
        norm_accel=self.normalize(raw_accel)

        raw_gyro=np.array([[line_u[3]],[line_u[4]],[line_u[5]]])
        omega_measured=self.convert_gyro(raw_gyro)

        raw_magne=np.array([[line_u[6]],[line_u[7]],[line_u[8]]])
        norm_magne=self.normalize(raw_magne)

        return norm_accel, omega_measured, norm_magne

    def getAttitude(self,arg):
        b_d = np.array([[arg.mag_field[0]], [arg.mag_field[1]] , [arg.mag_field[2]]])

        ax,ay,az = arg.accel
        gx,gy,gz = arg.gyro
        mx,my,mz = arg.mag
        line=[ax,ay,az,gx,gy,gz,mx,my,mz]
        norm_accel,omega_measured,norm_magne = self.parseData(line)

        norm_accel = np.dot(self.C_em, norm_accel)
        omega_measured = np.dot(self.C_em, omega_measured)
        norm_magne = np.dot(self.C_em, norm_magne)

        '''
        if(self.firstRun):
            self.b_d=np.array([[norm_magne[0][0]],[norm_magne[1][0]],[norm_magne[2][0]]])
            self.g_i=np.array([[norm_accel[0][0]],[norm_accel[1][0]],[norm_accel[2][0]]])
            self.firstRun=False
        '''

        b_b=np.dot(self.Cbi_hat,self.b_d)
        g_b=np.dot(self.Cbi_hat,self.g_i)

        r=-5*(np.dot(self.cross(g_b),norm_accel) + 0.25*np.dot(self.cross(b_b),norm_magne))

        omega_hat = omega_measured+r
        omega_hat_mag = math.sqrt(omega_hat[0][0]**2+omega_hat[1][0]**2+omega_hat[2][0]**2)
        omega_hat_mag_r = 1.0/omega_hat_mag

        Ak = np.eye(3) - self.cross(omega_hat)*math.sin(omega_hat_mag*self.T_sample)*omega_hat_mag_r + (1-math.cos(omega_hat_mag*self.T_sample))*np.dot(self.cross(omega_hat),self.cross(omega_hat))*omega_hat_mag_r**2

        self.Cbi_hat = np.dot(Ak,self.Cbi_hat)

        euler=tf.euler_from_matrix(self.Cbi_hat.T, axes='szyx')
        euler=tuple([x*180.0/math.pi for x in euler])

        return euler, self.Cbi_hat
