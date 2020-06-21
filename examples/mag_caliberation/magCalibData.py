import smbus
import zmq
import numpy as np
import sys
import os
import time

from imusensor.MPU9250 import MPU9250

# initializing publisher
host = '192.168.0.104'
port = 8358
url = 'tcp://'+host+':'+str(port)
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(url)


address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.caliberateAccelerometer()
print ("Mag caliberation starting")
time.sleep(2)
# imu.caliberateMagApprox()
imu.caliberateMagPrecise()
print ("Mag caliberation Finished")
print (imu.MagBias)
print (imu.Magtransform)
print (imu.Mags)
imu.saveCalibDataToFile("/home/pi/calib_real_bolder.json")

while True:
	imu.readSensor()
	imu.computeOrientation()
	print ("Mag x: {0} Mag y: {1} Mag Z: {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
	md = dict(topic = 'mag', data = str(imu.MagVals.tolist()))
	socket.send_json(md)
	time.sleep(0.1)