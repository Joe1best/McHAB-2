import RPi.GPIO as GPIO
import Adafruit_MCP4725 as MCP4725

class control:
    def __init__(self,dac):
        self.dac = dac

    def go(self, voltage):
        if(voltage > 0):
            GPIO.output(4,GPIO.HIGH)
        else:
            GPIO.output(4,GPIO.LOW)
            voltage=-voltage
        self.dac.setVoltage(int(voltage*4095/5.0))

