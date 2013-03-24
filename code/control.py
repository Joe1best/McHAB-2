import RPi.GPIO as GPIO
import Adafruit_MCP4725 as MCP4725

class control:
    def __init__(self,dac):
        self.dac = dac

    def control(self, voltage):
        if(voltage > 0):
            GPIO.output()

