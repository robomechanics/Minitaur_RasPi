#
# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
#
# Command: Sends BehaviorCmds to the robot to command its behavior.

import struct
import serial
import math
#from time import sleep
import time
import ctypes as ct
import socket
import array
import sys
import signal
import Adafruit_ADS1x15
import subprocess

adc = Adafruit_ADS1x15.ADS1115()

#USER INPUTS 
tcpPort = 50000
nVariables = 10 #Assumes all variables are of type double
debug = True

#SERIAL STUFF
# Open USB port
portAddress = "/dev/ttyS0"
#portAddress = "/dev/tty.usbserial-DN01QAKV"
#portAddress = "COM29"
baud = 115200
# baud = 230400
# No timeout so we can keep going with whatever bytes are in waiting
port = serial.Serial(portAddress, baud, timeout=None)
print("Sending to: " + port.name)

#TCP STUFF
#Init last commands to zero
mDataLast = [0.0]*nVariables
#mDataLast = mData = [0.0, 0.0, 2.0, 0.45, 0.0, 0.0, 0.0, 0.0, 0.0]
#Setup TCP connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', tcpPort)) #for anyone to connect to
s.listen(1)
print ("waiting for response from client at port ",tcpPort)
#function hangs until connected to by Matlab
conn, addr = s.accept()
print ('Connected by', addr)

#init values
rxBuf = b''
adc01 = adc23 = adc01Last = adc23Last = 0.0
sizzleFlag = 0.0
motorTemp = 0.0
startTime = time.time()
totEnergy = float(0.0)
pTimeLast = startTime
pTime = 0.0
#Init main while loop variable to True
run = True

def calculateChecksum(bytes, length):
    checksum = ct.c_ushort(0)
    for i in range(length):
        checksum = ct.c_ushort(int(checksum.value) + bytes[i]);
    return checksum.value

# Handle for getting Ctrl+C input, exits main while loop
def exitHandle(signal, frame):
    global run
    print('\nClosing connection and exiting function')
    run = False

print('Press Ctrl+C to exit function')

while(run):

    #Check if user wants to end function
    signal.signal(signal.SIGINT, exitHandle)


    #Read data from TCP connection
    tcpData = conn.recv(1024)
    chunks = tcpData.split(b'RM')
    for chunk in chunks[::-1]:
        if len(chunk) > 0:
            try:
                #Parse data from Main Computer(fwdvel,yawCmd,opt vars 1-7)
                newData = struct.unpack('>9fH',chunk)

                # Compare checksum
                checksum = newData[nVariables-1]
                checksumCalculated = calculateChecksum(chunk, 4*9) #9 Floats
                if(checksum == checksumCalculated):
                    #Save State Data
                    mData = newData[:nVariables-1:1]
                    break

            except:
                mData = mDataLast
                continue
        else:
            mData = mDataLast

    #Assign tcp data to variables (for readability)
    fwdVel = mData[0]
    yaw = mData[1]

    var1 = mData[2]
    var2 = mData[3]
    var3 = mData[4]
    var4 = mData[5]
    var5 = mData[6]
    var6 = mData[7]
    var7 = mData[8]

    # Custom packet for hardware in the loop optimizations
    behavior_command = struct.pack( '<I2f4f3f', 1, # Version 1 of serial packet format
                                                    fwdVel,yaw,
                                                    var1,var2,var3,var4,
                                                    var5,var6,var7)


    # Calculate and append checksum, prepend align bytes
    checksum = calculateChecksum(behavior_command, len(behavior_command))
    behavior_command = struct.pack('<cc', b"G", b"R") + behavior_command + struct.pack('<H', checksum)

    try:
        #Send BehaviorCmd via serial
        port.write(behavior_command)
    except:
        print('ElliePi failed to send data to Minitaur!!!, moving on')
        continue

    #Try to get Power Consumption Data
    try:
    	adc01 = adc.read_adc_difference(0, gain=8, data_rate=860)
    	time.sleep(0.001)
    	adc23 = adc.read_adc_difference(3, gain=8, data_rate=860)
    	pTime = time.time() - startTime
    except IOError:
        print("Restarting Power Monitoring Board!!!")
        subprocess.call(['i2cdetect', '-y', '1'])
        adc01 = adc01Last
        adc23 = adc23Last
        pTime = pTimeLast
        continue

    #Calc current and Voltage
    voltageDiff1 = adc01 * 0.00001562547
    voltageDiff2 = adc23 * 0.00001562547
    current = voltageDiff1 * -1000
    voltage = voltageDiff2 * 39.4039

    dt = pTime - pTimeLast

    if (dt > 0):
    	totEnergy += voltage*current*dt
    else:
    	print('dt value = ',dt)

    #save last values
    adc01Last = adc01
    adc23Last = adc23

    #setup return packet to main computer
    rtnPack = struct.pack('<3f',totEnergy, pTime, motorTemp)
    rtnChecksum = calculateChecksum(rtnPack, 3*4) #3 floats
    # Pack data into byte array for binary send
    powerData = struct.pack('>cc',b"R", b"M") + rtnPack + struct.pack('<H',rtnChecksum)

    # Send data via socket connection
    try:
        conn.send(powerData)
    except:
        print('ElliePi failed to send data to Main Computer!!!, moving on')
        continue

    print('Energy Data = ',totEnergy,', at time: ',pTime)

    # Debug Loop: Read incoming data
    rxBuf = port.read(50)
    chunks = rxBuf.split(b'GR')
    for chunk in chunks[::-1]:
        if len(chunk) > 0:
            try:
                # Parse packet into tuple: time, lastRX time, 3 float robot state, checksum 
                tup = struct.unpack('<3IfH', chunk)
            except:
                continue

            # Get times
            RXtime = tup[0]
            lastRXtime = tup[1]

            # Compare checksum
            checksum = tup[4]
            checksumCalculated = calculateChecksum(b'GR' + chunk, 2+4+4+4+4)
            # Print state
            if(checksum == checksumCalculated):
                #print("Times: " + str(RXtime) + " " + str(lastRXtime) + " " + str(RXtime-lastRXtime) + " Robot Sizzle state: " + str(tup[2]))
                sizzleFlag = float(tup[2])
                motorTemp = tup[3]
                exitFlag = True
                break
        else:
            continue
            # Put back the last (possibly unfinished) chunk
            # rxBuf = chunks[-1]
    # Save last commands from Matlab
    mDataLast = mData
    pTimeLast = pTime

# Close Conections
port.close()
conn.close()
print('Connection closed')


# BehaviorCmd definition:

# typedef struct _BehaviorCmd {
#     uint32_t id;
#     Twist twist;
#     Pose pose;
#     uint32_t mode;
# } BehaviorCmd;

# typedef struct _Twist {
#     Vector3 linear;
#     Vector3 angular;
# } Twist;

# typedef struct _Pose {
#     Vector3 position;
#     Quaternion orientation;
# } Pose;

# typedef struct _Vector3 {
#     float x;
#     float y;
#     float z;
# } Vector3;

# typedef struct _Quaternion {
#     float x;
#     float y;
#     float z;
#     float w;
# } Quaternion;
