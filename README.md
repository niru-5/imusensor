# imu-mpu9250
The repo provides a bridge between MPU9250 and raspberry pi. It also lists various caliberation code and filters for getting an accurate orientation from MPU9250
This repo mostly concentrates on the problem of connecting IMU(MPU9250) to raspberry pi through I2C communication. 
# Pre-requisites
Some of the requirements are to enable I2C in rpi. 
Installing I2C tools and smbus
```bash
sudo apt-get install i2c-tools
sudo pip install smbus
```
Connect the MPU9250 with rpi using the below connections
| Rpi pin | MPU9250 pins |
| ------ | ---------|
| pin 3 ->| SDA pin |
| pin 5 ->| SCL pin |
| pin 6 ->| Ground(GND) |
| pin 1 ->| VCC |

After you have made the connections, type the following command - 
```bash
sudo i2cdetect -y 1
```
If you see 68 in the output, then that means the sensor is connected to the rpi and 68 is the address of the sensor. 

# Basic Usage
The below code is a basic starter for the library
```python
import smbus
import numpy as np
from MPU9250 import MPU9250
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.caliberateGyro()
imu.caliberateAccelerometer()
# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/pi/calib.json")

while True:
	imu.readSensor()
	accel_vals = imu.AccelVals() # 3*1 array with x-axis, y-axis and z-axis
  gyro_vals = imu.GyroVals() # 3*1 array with x-axis, y-axis and z-axis
  mag_vals = imu.MagVals() # 3*1 array with x-axis, y-axis and z-axis

	print ("Acc: {0} ; Gyro : {1} ; Mag : {2}".format(accel_vals, gyro_vals, mag_vals))
	time.sleep(0.1)

```
# Other Functionalities

## Setting Accelerometer Range
The accelerometer in MPU9250 has the following ranges of +-2g, +-4g, +-8g and +-16g  
You can set this setting by the below command
```python
imu.setAccelRange("AccelRangeSelect2G")
```
Simiarly for 4g use "AccelRangeSelect4G" and follow similary for 8g and 16g ranges.

## Setting Gyroscope Range
Gyroscope sensor in MPU9250 has the following ranges +-250DPS, +-500DPS, +-1000DPS and +-2000DPS  
You can set this setting by the below command
```python
imu.setGyroRange("GyroRangeSelect250DPS")
```
Simiarly for 500DPS use "GyroRangeSelect500DPS" and follow similary for 1000DPS and 2000DPS ranges.  
**Note:** DPS means degrees per second

## Setting internal low pass filter frequency

## Gyroscope Caliberation

## Accelerometer Caliberation

## Magnometer Caliberation

## IMU Orientation

## Filters 
## Kalman
## Madgwick

## Filter comparison

# Credits
