import numpy as np
import os
import sys
import time
from json import JSONEncoder
import json
import smbus

from imusensor.MPU9250 import config

class MPU9250:
	"""
	An interface between MPU9250 and rpi using I2C protocol

	It has various fuctions from caliberation to computing orientation

	"""

	def __init__(self, bus, address):
		"""
		Sets up the basic variables like scale and bias of sensors.

		"""

		self.cfg = config.getConfigVals()
		self.cfg.Address = address
		self.Bus = bus
		self.AccelBias = np.array([0.0, 0.0, 0.0])
		self.Accels = np.array([1.0, 1.0, 1.0])
		self.MagBias = np.array([0.0, 0.0, 0.0])
		self.Mags = np.array([1.0, 1.0, 1.0])
		self.GyroBias = np.array([0.0, 0.0, 0.0])
		self.Magtransform = None

	def begin(self):
		"""
		Initializes various registers of MPU9250.

		It also sets ranges of accelerometer and gyroscope and also the frequency of low 
		pass filter.

		"""

		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)
		self.__writeRegister(self.cfg.UserControl, self.cfg.I2CMasterEnable)
		self.__writeRegister(self.cfg.I2CMasterControl, self.cfg.I2CMasterClock)

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
		#self.__writeRegister(self.cfg.PowerManagement1, self.cfg.PowerReset) # power is not reseting
		time.sleep(0.01)
		#self.__writeAK8963Register(self.cfg.Ak8963CNTL2, self.cfg.Ak8963Reset) # AK8963 is not resetting
		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)

		name = self.__whoAmI()
		if not (name[0] == 113 or name[0] == 115 ):
			print ("The name is wrong {0}".format(name))
		self.__writeRegister(self.cfg.PowerManagement2, self.cfg.SensorEnable)

		self.setAccelRange("AccelRangeSelect16G")

		self.setGyroRange("GyroRangeSelect2000DPS")

		self.setLowPassFilterFrequency("AccelLowPassFilter184")

		self.__writeRegister(self.cfg.SMPDivider, 0x00)
		self.CurrentSRD = 0x00
		# self.setSRD(0x00)

		self.__writeRegister(self.cfg.UserControl, self.cfg.I2CMasterEnable)
		self.__writeRegister(self.cfg.I2CMasterControl, self.cfg.I2CMasterClock)

		magName = self.__whoAmIAK8963() # mag name seems to be different
		if magName[0] != 72:
			print ("The mag name is different and it is {0}".format(magName))

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963FuseROM)
		time.sleep(0.1)
		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)
		time.sleep(0.1)
		self.MagScale = self.__readAK8963Registers(self.cfg.Ak8963ASA, 3)
		self.MagScale = np.array(self.MagScale)
		self.MagScale = ((self.MagScale - 128.0)/256.0 + 1.0)*(4912.0/32760.0)

		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
		time.sleep(0.1)
		self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)
		time.sleep(0.1)

		self.__writeRegister(self.cfg.PowerManagement1, self.cfg.ClockPLL)
		self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)

		# Caliberating Gyro 
		self.caliberateGyro()

		return 1

	def setSRD(self, data):
		"""Sets the frequency of getting data

		Parameters
		----------
		data : int
			This number is between 1 to 19 and decides the rate of sample collection

		"""

		self.CurrentSRD = data
		self.__writeRegister(self.cfg.SMPDivider, 19)

		if data > 9:
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
			time.sleep(0.1)
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment1)
			time.sleep(0.1)
			self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)
		else:
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963PowerDown)
			time.sleep(0.1)
			self.__writeAK8963Register(self.cfg.Ak8963CNTL1, self.cfg.Ak8963ContinuosMeasurment2)
			time.sleep(0.1)
			self.__readAK8963Registers(self.cfg.Ak8963HXL, 7)

		self.__writeRegister(self.cfg.SMPDivider, data)

	def setAccelRange(self, accelRange):
		"""Sets the range of accelerometer

		Parameters
		----------
		accelRange : str
			The supported ranges are as following ->
			2g  -> AccelRangeSelect2G
			4g  -> AccelRangeSelect4G
			8g  -> AccelRangeSelect8G
			16g -> AccelRangeSelect16G

		"""
		
		try:
			self.__writeRegister(self.cfg.AccelConfig, self.cfg[accelRange])
			self.AccelRange = accelRange
		except:
			print ("{0} is not a proper value for accelerometer range".format(accelRange))
			return -1
		accelVal = float(accelRange.split('t')[1].split('G')[0])
		self.AccelScale = self.cfg.Gravity*accelVal/32767.5
		return 1

	def setGyroRange(self, gyroRange):
		"""Sets the range of gyroscope

		Parameters
		----------
		gyroRange : str
			The supported ranges are as following ->
			250DPS  -> GyroRangeSelect250DPS
			500DPS  -> GyroRangeSelect500DPS
			1000DPS -> GyroRangeSelect1000DPS
			2000DPS -> GyroRangeSelect2000DPS

			DPS means degrees per freedom

		"""

		try:
			self.__writeRegister(self.cfg.GyroConfig, self.cfg[gyroRange])
			self.GyroRange = gyroRange
		except:
			print ("{0} is not a proper value for gyroscope range".format(gyroscope))
			return -1
		gyroVal = float(gyroRange.split('t')[1].split('D')[0])
		self.GyroScale = self.cfg.Degree2Radian*(gyroVal/32767.5)
		return 1

	def setLowPassFilterFrequency(self, frequency):
		"""Sets the frequency of internal low pass filter

		This is common for both accelerometer and gyroscope

		Parameters
		----------
		frequency : str
			The supported frequencies are as following ->
			250DPS  -> GyroRangeSelect250DPS
			500DPS  -> GyroRangeSelect500DPS
			1000DPS -> GyroRangeSelect1000DPS
			2000DPS -> GyroRangeSelect2000DPS

			DPS means degrees per freedom

		"""

		try:
			self.__writeRegister(self.cfg.AccelConfig2, self.cfg[frequency])
			self.__writeRegister(self.cfg.GyroConfig2, self.cfg[frequency])
			self.Frequency = frequency
		except:
			print ("{0} is not a proper value forlow pass filter".format(frequency))
			return -1
		return 1

	def readRawSensor(self):
		"""Reading raw values of accelerometer, gyroscope and magnetometer

		"""

		data = self.__readRegisters(self.cfg.AccelOut, 20)

		data = np.array(data).astype(np.int16)
		highbits = data[::2]<<8
		vals = highbits + data[1::2]

		self.RawAccelVals = np.squeeze(self.cfg.transformationMatrix.dot((vals[np.newaxis,:3].T)))*self.AccelScale
		self.RawGyroVals = np.squeeze(self.cfg.transformationMatrix.dot((vals[np.newaxis,4:7].T)))*self.GyroScale
		self.RawMagVals = (vals[-3:])*self.MagScale 
		self.RawTemp = vals[3]

	def readSensor(self):
		"""Reading values of accelerometer, gyroscope and magnetometer 

		The functions finds values by applying caliberation values.

		"""

		data = self.__readRegisters(self.cfg.AccelOut, 21)

		data = np.array(data[:-1]).astype(np.int16)
		magData = data[14:]
		highbits = data[::2]<<8
		vals = highbits + data[1::2]
		magHighbits = magData[1::2]<<8
		magvals = magHighbits + magData[::2]

		self.AccelVals = (np.squeeze(self.cfg.transformationMatrix.dot((vals[np.newaxis,:3].T)))*self.AccelScale - self.AccelBias)*self.Accels
		self.GyroVals = np.squeeze(self.cfg.transformationMatrix.dot((vals[np.newaxis,4:7].T)))*self.GyroScale - self.GyroBias

		if self.Magtransform is None:
			self.MagVals = ((magvals[-3:])*self.MagScale - self.MagBias)*self.Mags
		else:
			self.MagVals = np.matmul((magvals[-3:])*self.MagScale - self.MagBias, self.Magtransform)

		self.Temp = (vals[3] - self.cfg.TempOffset)/self.cfg.TempScale + self.cfg.TempOffset

	def caliberateGyro(self):
		"""Calibrates gyroscope by finding the bias sets the gyro bias

		"""

		currentGyroRange = self.GyroRange
		currentFrequency = self.Frequency
		currentSRD = self.CurrentSRD
		self.setGyroRange("GyroRangeSelect250DPS")
		self.setLowPassFilterFrequency("AccelLowPassFilter20")
		self.setSRD(19)

		gyroBias1 = np.array([0.0,0.0,0.0])
		for i in range(100):
			self.readSensor()
			gyroBias1 = gyroBias1 + self.GyroBias + self.GyroVals
			time.sleep(0.02)

		self.GyroBias = gyroBias1/100.0

		self.setGyroRange(currentGyroRange)
		self.setLowPassFilterFrequency(currentFrequency)
		self.setSRD(currentSRD)

	def caliberateAccelerometer(self):
		"""Caliberate Accelerometer by positioning it in 6 different positions
		
		This function expects the user to keep the imu in 6 different positions while caliberation. 
		It gives cues on when to change the position. It is expected that in all the 6 positions, 
		at least one axis of IMU is parallel to gravity of earth and no position is same. Hence we 
		get 6 positions namely -> +x, -x, +y, -y, +z, -z.
		"""

		currentAccelRange = self.AccelRange
		currentFrequency = self.Frequency
		currentSRD = self.CurrentSRD
		self.setAccelRange("AccelRangeSelect2G")
		self.setLowPassFilterFrequency("AccelLowPassFilter20")
		self.setSRD(19)

		xbias = []
		ybias = []
		zbias = []
		xscale = []
		yscale = []
		zscale = []

		print ("Acceleration calibration is starting and keep placing the IMU in 6 different directions based on the instructions below")
		time.sleep(2)
		for i in range(6):
			input("Put the IMU in {0} position. Press enter to continue..".format(i+1))
			time.sleep(3)
			meanvals = self.__getAccelVals()
			print (meanvals)
			xscale, xbias = self.__assignBiasOrScale(meanvals[0], xscale, xbias)
			yscale, ybias = self.__assignBiasOrScale(meanvals[1], yscale, ybias)
			zscale, zbias = self.__assignBiasOrScale(meanvals[2], zscale, zbias)
			print (xscale)
			print (yscale)
			print (zscale)

		if len(xscale) != 2 or len(yscale) != 2 or len(zscale) != 2:
			print ("It looks like there were some external forces on sensor and couldn't get proper values. Please try again")
			return


		self.AccelBias[0] = -1*(xscale[0] + xscale[1])/(abs(xscale[0]) + abs(xscale[1]))
		self.AccelBias[1] = -1*(yscale[0] + yscale[1])/(abs(yscale[0]) + abs(yscale[1]))
		self.AccelBias[2] = -1*(zscale[0] + zscale[1])/(abs(zscale[0]) + abs(zscale[1]))

		self.AccelBias = -1*self.cfg.Gravity*self.AccelBias

		self.Accels[0] = (2.0*self.cfg.Gravity)/(abs(xscale[0]) + abs(xscale[1]))
		self.Accels[1] = (2.0*self.cfg.Gravity)/(abs(yscale[0]) + abs(yscale[1]))
		self.Accels[2] = (2.0*self.cfg.Gravity)/(abs(zscale[0]) + abs(zscale[1]))
		
		self.setAccelRange(currentAccelRange)
		self.setLowPassFilterFrequency(currentFrequency)
		self.setSRD(currentSRD)

	def __getScale(self, scale):
		if len(scale) == 0:
			return 1
		else:
			return sum(scale)/(2*self.cfg.Gravity)

	def __assignBiasOrScale(self, val, scale, bias):

		if val > 6.0 or val < -6.0 :
			scale.append(val)
		else:
			bias.append(val)
		return scale, bias


	def __getAccelVals(self):

		accelvals = np.zeros((100,3))
		for samples in range(1,100):
			self.readSensor()
			vals = self.AccelVals/self.Accels + self.AccelBias
			accelvals[samples] = vals
			time.sleep(0.02)
		meanvals = np.array([accelvals[:,0].mean(), accelvals[:,1].mean(), accelvals[:,2].mean()])
		return meanvals

	def caliberateMagApprox(self):
		"""Caliberate Magnetometer
		
		This function uses basic methods like averaging and scaling to find the hard iron
		and soft iron effects.

		Note: Make sure you rotate the sensor in 8 shape and cover all the 
		pitch and roll angles.

		"""

		currentSRD = self.CurrentSRD
		self.setSRD(19)
		numSamples = 1000
		magvals = np.zeros((numSamples,3))
		for sample in range(1,numSamples):
			self.readSensor()
			magvals[sample] = self.MagVals/self.Mags + self.MagBias
			time.sleep(0.02)
		minvals = np.array([magvals[:,0].min(), magvals[:,1].min(), magvals[:,2].min()])
		maxvals = np.array([magvals[:,0].max(), magvals[:,1].max(), magvals[:,2].max()])

		self.MagBias = (minvals + maxvals)/2.0
		averageRad = (((maxvals - minvals)/2.0).sum())/3.0
		self.Mags = ((maxvals - minvals)/2.0)*(1/averageRad)

		self.setSRD(currentSRD)

	def caliberateMagPrecise(self):
		"""Caliberate Magnetometer Use this method for more precise calculation
		
		This function uses ellipsoid fitting to get an estimate of the bias and
		transformation matrix required for mag data

		Note: Make sure you rotate the sensor in 8 shape and cover all the 
		pitch and roll angles.

		"""

		currentSRD = self.CurrentSRD
		self.setSRD(19)
		numSamples = 1000
		magvals = np.zeros((numSamples,3))
		for sample in range(1,numSamples):
			self.readSensor()
			magvals[sample] = self.MagVals/self.Mags + self.MagBias
			time.sleep(0.05)
		centre, evecs, radii, v = self.__ellipsoid_fit(magvals)

		a, b, c = radii
		r = (a * b * c) ** (1. / 3.)
		D = np.array([[r/a, 0., 0.], [0., r/b, 0.], [0., 0., r/c]])
		transformation = evecs.dot(D).dot(evecs.T)

		self.MagBias = centre
		self.Magtransform = transformation

		self.setSRD(currentSRD)

	def __ellipsoid_fit(self, X):
		x = X[:, 0]
		y = X[:, 1]
		z = X[:, 2]
		D = np.array([x * x + y * y - 2 * z * z,
					x * x + z * z - 2 * y * y,
					2 * x * y,
					2 * x * z,
					2 * y * z,
					2 * x,
					2 * y,
					2 * z,
					1 - 0 * x])
		d2 = np.array(x * x + y * y + z * z).T # rhs for LLSQ
		u = np.linalg.solve(D.dot(D.T), D.dot(d2))
		a = np.array([u[0] + 1 * u[1] - 1])
		b = np.array([u[0] - 2 * u[1] - 1])
		c = np.array([u[1] - 2 * u[0] - 1])
		v = np.concatenate([a, b, c, u[2:]], axis=0).flatten()
		A = np.array([[v[0], v[3], v[4], v[6]],
					[v[3], v[1], v[5], v[7]],
					[v[4], v[5], v[2], v[8]],
					[v[6], v[7], v[8], v[9]]])

		center = np.linalg.solve(- A[:3, :3], v[6:9])

		translation_matrix = np.eye(4)
		translation_matrix[3, :3] = center.T

		R = translation_matrix.dot(A).dot(translation_matrix.T)

		evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
		evecs = evecs.T

		radii = np.sqrt(1. / np.abs(evals))
		radii *= np.sign(evals)

		return center, evecs, radii, v

	def saveCalibDataToFile(self, filePath):
		""" Save the caliberation vaslues

		Parameters
		----------
		filePath : str
			Make sure the folder exists before giving the input.  The path 
			has to be absolute.
			Otherwise it doesn't save the values.

		"""

		calibVals = {}
		calibVals['Accels'] = self.Accels
		calibVals['AccelBias'] = self.AccelBias
		calibVals['GyroBias'] = self.GyroBias
		calibVals['Mags'] = self.Mags
		calibVals['MagBias'] = self.MagBias
		if self.Magtransform is not None:
			calibVals['Magtransform'] = self.Magtransform

		# check if folder of the path exists
		dirName = os.path.dirname(filePath)
		if not os.path.isdir(dirName):
			print ("Please provide a valid folder")
			return
		basename = os.path.basename(filePath)
		if basename.split('.')[-1] != 'json':
			print ("Please provide a json file")
			return

		with open(filePath, 'w') as outFile:
			json.dump(calibVals, outFile, cls =NumpyArrayEncoder)

	def loadCalibDataFromFile(self, filePath):
		""" Save the caliberation vaslues

		Parameters
		----------
		filePath : str
			Make sure the file exists before giving the input. The path 
			has to be absolute.
			Otherwise it doesn't save the values.
		
		"""

		#check if file path exists
		if not os.path.exists(filePath):
			print ("Please provide the correct path")

		with open(filePath, 'r') as jsonFile:
			calibVals = json.load(jsonFile)
			self.Accels = np.asarray(calibVals['Accels'])
			self.AccelBias = np.asarray(calibVals['AccelBias'])
			self.GyroBias = np.asarray(calibVals['GyroBias'])
			self.Mags = np.asarray(calibVals['Mags'])
			self.MagBias = np.asarray(calibVals['MagBias'])
			if 'Magtransform' in calibVals.keys():
				self.Magtransform = np.asarray(calibVals['Magtransform'])

	def computeOrientation(self):
		""" Computes roll, pitch and yaw

		The function uses accelerometer and magnetometer values
		to estimate roll, pitch and yaw. These values could be 
		having some noise, hence look at kalman and madgwick 
		filters in filters folder to get a better estimate.
		
		"""

		self.roll = np.arctan2(self.AccelVals[1], self.AccelVals[2] + 0.05*self.AccelVals[0])
		self.pitch = np.arctan2(-1*self.AccelVals[0], np.sqrt(np.square(self.AccelVals[1]) + np.square(self.AccelVals[2]) ))
		magLength = np.sqrt(np.square(self.MagVals).sum())
		normMagVals = self.MagVals/magLength
		self.yaw = np.arctan2(np.sin(self.roll)*normMagVals[2] - np.cos(self.roll)*normMagVals[1],\
					np.cos(self.pitch)*normMagVals[0] + np.sin(self.roll)*np.sin(self.pitch)*normMagVals[1] \
					+ np.cos(self.roll)*np.sin(self.pitch)*normMagVals[2])

		self.roll = np.degrees(self.roll)
		self.pitch = np.degrees(self.pitch)
		self.yaw = np.degrees(self.yaw)

	def __writeRegister(self, subaddress, data):

		self.Bus.write_byte_data(self.cfg.Address, subaddress, data)
		time.sleep(0.01)

		val = self.__readRegisters(subaddress,1)
		if val[0] != data:
			print ("It did not write the {0} to the register {1}".format(data, subaddress))
			return -1
		return 1

	def __readRegisters(self, subaddress, count):

		data = self.Bus.read_i2c_block_data(self.cfg.Address, subaddress, count)
		return data

	def __writeAK8963Register(self, subaddress, data):

		self.__writeRegister(self.cfg.I2CSlave0Address, self.cfg.Ak8963I2CAddress)
		self.__writeRegister(self.cfg.I2CSlave0Register, subaddress)
		self.__writeRegister(self.cfg.I2CSlave0Do, data)
		self.__writeRegister(self.cfg.I2CSlave0Control, self.cfg.I2CSlave0Enable | 1)

		val = self.__readAK8963Registers(subaddress, 1)

		if val[0] != data:
			print ("looks like it did not write properly")
		return 1

	def __readAK8963Registers(self, subaddress, count):

		self.__writeRegister(self.cfg.I2CSlave0Address, self.cfg.Ak8963I2CAddress | self.cfg.I2CReadFlad)
		self.__writeRegister(self.cfg.I2CSlave0Register, subaddress)
		self.__writeRegister(self.cfg.I2CSlave0Control, self.cfg.I2CSlave0Enable | count)

		time.sleep(0.01)
		data = self.__readRegisters(self.cfg.ExtSensData00, count)
		return data

	def __whoAmI(self):

		data = self.__readRegisters(self.cfg.WhoAmI, 1)
		return data

	def __whoAmIAK8963(self):

		data = self.__readAK8963Registers(self.cfg.Ak8963WhoAmI, 1)
		return data

	@property
	def roll(self):
		return self._roll

	@roll.setter
	def roll(self, roll):
		self._roll = roll

	@property
	def pitch(self):
		return self._pitch

	@pitch.setter
	def pitch(self, pitch):
		self._pitch = pitch

	@property
	def yaw(self):
		return self._yaw

	@yaw.setter
	def yaw(self, yaw):
		self._yaw = yaw

	@property
	def Bus(self):
		return self._Bus

	@Bus.setter
	def Bus(self, Bus):
		if isinstance(Bus, smbus.SMBus):
			self._Bus = Bus
		else:
			raise Exception("Please provide the object created by smbus")


class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)
