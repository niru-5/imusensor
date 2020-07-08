import smbus
import numpy as np
import sys
import os
import time

from imusensor.MPU9250 import MPU9250


address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
print ("Accel calibration starting")
imu.caliberateAccelerometer()
print ("Accel calibration Finisehd")
print (imu.AccelBias)
print (imu.Accels)

print ("Mag calibration starting")
time.sleep(2)
# imu.caliberateMagApprox()
imu.caliberateMagPrecise()
print ("Mag calibration Finished")
print (imu.MagBias)
print (imu.Magtransform)
print (imu.Mags)
imu.saveCalibDataToFile("/home/pi/calib_real4.json")