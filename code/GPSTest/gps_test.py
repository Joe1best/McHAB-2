import pygame
from pygame.locals import *
import time
import serial

ser = serial.Serial('COM6', 4800, timeout=1)

pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Pygame Caption')
pygame.mouse.set_visible(0)
lat = 4542.0
long = 7311.0 
height = 0.0
 
done = False
while not done:
    for event in pygame.event.get():
        if(event.type==KEYDOWN):
            if (event.key == K_UP):
                lat=lat+10

            if (event.key == K_DOWN):
                lat=lat-10
                
            if (event.key == K_LEFT):
                long=long-10

            if (event.key == K_RIGHT):
                long=long+10

            if (event.key == K_ESCAPE):
                done = True
                
            if (event.key == K_SPACE):
                height=10000
            
                
    ser.write('$GPGGA,152105.000,'+str(lat) +',N,0'+ str(long) + ',W,1,10,0.8,'+str(height)+',M,-31.4,M,,0000*51\n')
    time.sleep(1.0)