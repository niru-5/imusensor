import os
import sys
import time
import smbus
import numpy as np

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
# imu.caliberateAccelerometer()
# print ("Acceleration calib successful")
# imu.caliberateMag()
# print ("Mag calib successful")
# or load your calibration file
# imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

sensorfusion = kalman.Kalman()

imu.readSensor()
imu.computeOrientation()
sensorfusion.roll = imu.roll
sensorfusion.pitch = imu.pitch
sensorfusion.yaw = imu.yaw

count = 0
currTime = time.time()
while True:
	imu.readSensor()
	imu.computeOrientation()
	newTime = time.time()
	dt = newTime - currTime
	currTime = newTime

	sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],\
												imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

	print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))

	time.sleep(0.01)