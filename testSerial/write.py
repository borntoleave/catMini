#!/usr/bin/env python


import time
import serial
import array
import struct
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM) # Set pin numbering to board numbering


ser = serial.Serial(
  
   port='/dev/ttyS0',
   baudrate = 9600,
   #parity=serial.PARITY_NONE,
   #stopbits=serial.STOPBITS_ONE,
   #bytesize=serial.EIGHTBITS,
   timeout=1
)


counter=0
calib=[1,2,3,-5,-90,60,4,3,1,2,3,2,3,5,3,4];

def serialWriteByte(instrType,var):
   headTable={'calib':0,\
               'pose':1,\
               'mode':2,}
   packetHead=headTable[instrType]
   if packetHead<2:
      instrStr=struct.pack('b'*17,packetHead,*var)
   else:#mode
      instrStr=str(packetHead)+var
   ser.write(instrStr)
   ser.write("\n")
   

try:
   GPIO_arduino=18
   GPIO.setup(GPIO_arduino,GPIO.OUT)
   GPIO.output(GPIO_arduino,True)
   while 1:
      serialWriteByte('pose',calib)
      if ser.in_waiting:
         x=ser.readline()
         if x!="":
            print (x+"\n")
      time.sleep(0.5)
      serialWriteByte('calib',calib)
      if ser.in_waiting:
         x=ser.readline()
         if x!="":
            print (x+"\n")
      time.sleep(0.5)
      serialWriteByte('mode',"walk")
      
      
      if ser.in_waiting:
         x=ser.readline()
         if x!="":
            print (x+"\n")
      time.sleep(1)
      counter += 1
except KeyboardInterrupt:
   print ("cleaning...")
   GPIO.output(GPIO_arduino,False)
   
