#!/usr/bin/env python
# coding: utf-8


import serial
import time

ser=serial.Serial('/dev/ttyACM0',9600, bytesize = 8, stopbits = 1, 
                  timeout = 0, parity='N')


# In[11]:


print(ser.name)         # check which port was really used

while True:
    cmd = input("Send Cmmds: ")
    if(cmd == 'ALRM'):
        ser.write(b'ALRM')
    elif (cmd == 'SEND'):
        
        ser.write(b'SEND')
        while True:
                       
            data = ser.read()           # Wait forever for anything
            time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
            data_left = ser.inWaiting()  # Get the number of characters ready to be read
            data += ser.read(data_left) # Do the read and combine it with the first character

            # INFO: https://stackoverflow.com/questions/13017840/using-pyserial-is-it-possible-to-wait-for-data

            if len(data) >= 10:
                print(data)
                break
