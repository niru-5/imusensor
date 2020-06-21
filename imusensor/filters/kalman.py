import numpy as np

class Kalman:
	"""
	Kalman filter for sensor fusion of IMU

	The class fuses the roll, pitch and yaw from accelrometer
	and magneotmeter with gyroscope. 
	refer to examples of the git repo

	"""
	def __init__(self):
		"""
		Initialises all the variables. 

		The option of setting your own values is given in the form of 
		set functions

		"""

		self.currentRollState = np.vstack((0.0, 0.0)) # updates
		self.roll = 0 # updates
		self.rollCovariance = np.zeros((2,2)) # updates
		self.rollError = 0.001
		self.rollDriftError = 0.003
		self.rollMeasurementError = 0.03

		self.currentPitchState = np.vstack((0.0, 0.0)) # updates
		self.pitch = 0 # updates
		self.pitchCovariance = np.zeros((2,2)) # updates
		self.pitchError= 0.001
		self.pitchDriftError = 0.003
		self.pitchMeasurementError = 0.03

		self.currentYawState = np.vstack((0.0, 0.0)) # updates
		self.yaw = 0 # updates
		self.yawCovariance = np.zeros((2,2)) #updates
		self.yawError = 0.001
		self.yawDriftError = 0.003
		self.yawMeasurementError = 0.03

	def computeAndUpdateRollPitchYaw(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
		"""
		Computes roll, pitch and yaw

		Parameters
		----------
		ax: float 
			acceleration in x axis
		ay: float 
			acceleration in y axis
		az: float 
			acceleration in z axis
		gx: float 
			angular velocity about x axis
		gy: float 
			angular velocity about y axis
		gz: float 
			angular velocity about z axis
		mx: float 
			magnetic moment about x axis
		my: float 
			magnetic moment about y axis
		mz: float 
			magnetic moment about z axis
		dt: float
			time interval for kalman filter to be applied

		Note: It saves the roll, pitch and yaw in the class 
			properties itself. You can directly call them by
			classname.roll 

		"""

		measuredRoll, measuredPitch = self.computeRollAndPitch(ax,ay,az)
		measuredYaw = self.computeYaw(measuredRoll, measuredPitch, mx, my, mz)

		reset, gy = self.__restrictRollAndPitch(measuredRoll, measuredPitch, gy)
		# reset = 0
		if not reset:
			self.roll, self.currentRollState, self.rollCovariance = self.update(self.currentRollState, \
																measuredRoll, self.rollCovariance, \
																self.rollError, self.rollDriftError, \
																self.rollMeasurementError, gx, dt) 

		self.pitch, self.currentPitchState, self.pitchCovariance = self.update(self.currentPitchState, \
																	measuredPitch, self.pitchCovariance, \
																	self.pitchError, self.pitchDriftError, \
																	self.pitchMeasurementError, gy, dt) 

		self.yaw, self.currentYawState, self.yawCovariance = self.update(self.currentYawState, \
															measuredYaw, self.yawCovariance, \
															self.yawError, self.yawDriftError, \
															self.yawMeasurementError, gz, dt)

	def __restrictRollAndPitch(self, measuredRoll, measuredPitch, gy):

		reset = 0
		if (measuredRoll < -90 and self.roll > 90) or (measuredRoll > 90 and self.roll < -90):
			self.roll = measuredRoll
			reset = 1
		if abs(self.roll) > 90:
			gy = -1*gy
		return reset, gy


	def computeAndUpdateRollPitch(self, ax, ay, az, gx, gy, dt):
		"""
		Computes roll and pitch

		Parameters
		----------
		ax: float 
			acceleration in x axis
		ay: float 
			acceleration in y axis
		az: float 
			acceleration in z axis
		gx: float 
			angular velocity about x axis
		gy: float 
			angular velocity about y axis
		dt: float
			time interval for kalman filter to be applied

		Note: It saves the roll and pitch in the class 
			properties itself. You can directly call them by
			classname.roll 

		"""

		measuredRoll, measuredPitch = self.computeRollAndPitch(ax,ay,az)

		reset, gy = self.__restrictRollAndPitch(measuredRoll, measuredPitch, gy)

		if not reset:
			self.roll, self.currentRollState, self.rollCovariance = self.update(self.currentRollState, \
																measuredRoll, self.rollCovariance, \
																self.rollError, self.rollDriftError, \
																self.rollMeasurementError, gx, dt) 

		self.pitch, self.currentPitchState, self.pitchCovariance = self.update(self.currentPitchState, \
																	measuredPitch, self.pitchCovariance, \
																	self.pitchError, self.pitchDriftError, \
																	self.pitchMeasurementError, gy, dt)

	def updateRollPitchYaw(self, roll, pitch, yaw, gx, gy, gz, dt):
		"""
		Computes sensor fused roll, pitch and yaw

		Parameters
		----------
		roll: float 
			estimate obtained from accelerometer
		pitch: float 
			estimate obtained from accelerometer
		yaw: float 
			estimate obtained from magnetometer
		gx: float 
			angular velocity about x axis
		gy: float 
			angular velocity about y axis
		gz: float 
			angular velocity about z axis
		dt: float
			time interval for kalman filter to be applied

		Note: It saves the roll, pitch and yaw in the class 
			properties itself. You can directly call them by
			classname.roll 

		"""

		self.updateRollPitch(roll, pitch, gx, gy, dt)

		self.yaw, self.currentYawState, self.yawCovariance = self.update(self.currentYawState, \
															yaw, self.yawCovariance, \
															self.yawError, self.yawDriftError, \
															self.yawMeasurementError, gz, dt)

	def updateRollPitch(self, roll, pitch, gx, gy, dt):
		"""
		Computes sensor fused roll and pitch

		Parameters
		----------
		roll: float 
			estimate obtained from accelerometer
		pitch: float 
			estimate obtained from accelerometer
		gx: float 
			angular velocity about x axis
		gy: float 
			angular velocity about y axis
		dt: float
			time interval for kalman filter to be applied

		Note: It saves the roll and pitch  in the class 
			properties itself. You can directly call them by
			classname.roll 

		"""

		self.roll, self.currentRollState, self.rollCovariance = self.update(self.currentRollState, \
																roll, self.rollCovariance, \
																self.rollError, self.rollDriftError, \
																self.rollMeasurementError, gx, dt) 

		self.pitch, self.currentPitchState, self.pitchCovariance = self.update(self.currentPitchState, \
																	pitch, self.pitchCovariance, \
																	self.pitchError, self.pitchDriftError, \
																	self.pitchMeasurementError, gy, dt)

	def computeRollAndPitch(self, ax, ay, az):
		"""
		Computes measured roll and pitch from accelerometer

		Parameters
		----------
		ax: float 
			acceleration in x axis
		ay: float 
			acceleration in y axis
		az: float 
			acceleration in z axis

		Returns
		-------
		measuresRoll: float
					It is estimated roll from sensor values
		measuresPitch: float
					It is estimated pitch from sensor values

		"""

		measuredRoll = np.degrees(np.arctan2(ay,az))
		measuredPitch = np.degrees(np.arctan2(-1*ax, np.sqrt(np.square(ay) + np.square(az)) ) )

		return measuredRoll, measuredPitch

	def computeYaw(self, roll, pitch, mx, my, mz):
		"""
		Computes measured yaw

		Parameters
		----------
		roll: float 
			estimate obtained from accelerometer
		pitch: float 
			estimate obtained from accelerometer
		mx: float 
			magnetic moment about x axis
		my: float 
			magnetic moment about y axis
		mz: float 
			magnetic moment about z axis

		Returns
		-------
		measuresYaw: float
					It is estimated yaw from sensor values

		"""

		roll = np.radians(roll)
		pitch = np.radians(pitch)
		magLength = np.sqrt(sum([mx*mx + my*my + mz*mz]))
		mx = mx/magLength
		my = my/magLength
		mz = mz/magLength

		measuredYaw = np.degrees(np.arctan2(np.sin(roll)*mz - np.cos(roll)*my,\
					np.cos(pitch)*mx + np.sin(roll)*np.sin(pitch)*my \
					+ np.cos(roll)*np.sin(pitch)*mz) )

		return measuredYaw

	def update(self, currentState, measurement, currentCovariance, error, driftError, measurementError, angularVelocity ,dt):
		"""
		Core function of Kalman relating to its implmentation

		Parameters
		----------
		currentState: float array 
					It is current state of the sensor which implies current 
					orientation in a specific axis and its corresponding 
					bias. ex - [roll, roll_bias]
		measurement: float 
			estimate of the orinetation by the sensor. ex - measuredRoll
		currentCovariance: 2*2 array 
						This represents matrix relating orientation and bias
						ex - rollCovariance
		error: float
			This represents error in estimating the orientation
		driftError: float
				This represents error in estimating the  bias in orientation
		measurementError: float
						This represents error in sensor values
		angularVelocity: float
						The angular velocity about the direction
						of orientation
		dt: float
			time interval for kalman filter to be applied

		Returns
		-------
		orientation: float
					It is the corrected angle from previous
					estimate and current measurment
		correctedState:
					It is the corrected state from previous
					estimate and current measurment
		updatedCovariance: 
					New updated covariance after taking 
					new measurement into consideration

		"""

		motionModel = np.array([[1,-1*dt],[0,1]])

		prediction = np.matmul(motionModel,currentState) + dt*(np.vstack((angularVelocity,0.0)))

		errorMatrix = np.array([error, driftError])*np.identity(2)
		predictedCovariance = np.matmul(np.matmul(motionModel, currentCovariance), (motionModel.T)) + errorMatrix

		difference = measurement - np.matmul(np.array([1.0, 1.0]), prediction)

		measurementCovariance = np.matmul(np.matmul(np.array([1.0, 0.0]), predictedCovariance),np.vstack((1.0,0.0))) + measurementError
		kalmanGain = np.matmul(predictedCovariance, np.vstack((1.0, 0.0)))/measurementCovariance

		correctedState = prediction + kalmanGain*(measurement - np.matmul(np.array([1.0, 0.0]), prediction))
		# correctedState = prediction + kalmanGain*(difference)

		updatedCovariance = np.matmul( np.identity(2) - np.matmul(kalmanGain, np.array([1.0, 0.0]).reshape((1,2))), predictedCovariance)

		return correctedState[0,0], correctedState, updatedCovariance

	@property
	def roll(self):
		return self._roll

	@roll.setter
	def roll(self, roll):
		self._roll = roll
		self.currentRollState[0,0] = roll

	@property
	def pitch(self):
		return self._pitch

	@pitch.setter
	def pitch(self, pitch):
		self._pitch = pitch
		self.currentPitchState[0,0] = pitch

	@property
	def yaw(self):
		return self._yaw

	@yaw.setter
	def yaw(self, yaw):
		self._yaw = yaw
		self.currentYawState[0,0] = yaw

	@property
	def rollError(self):
		return self._rollError

	@rollError.setter
	def rollError(self, error):
		self._rollError = error

	@property
	def rollDriftError(self):
		return self._rollDriftError

	@rollDriftError.setter
	def rollDriftError(self, error):
		self._rollDriftError = error


	@property
	def rollMeasurementError(self):
		return self._rollMeasurementError

	@rollMeasurementError.setter
	def rollMeasurementError(self, error):
		self._rollMeasurementError = error

	@property
	def pitchError(self):
		return self._pitchError

	@pitchError.setter
	def pitchError(self, error):
		self._pitchError = error

	@property
	def pitchDriftError(self):
		return self._pitchDriftError

	@pitchDriftError.setter
	def pitchDriftError(self, error):
		self._pitchDriftError = error

	@property
	def pitchMeasurementError(self):
		return self._pitchMeasurementError

	@pitchMeasurementError.setter
	def pitchMeasurementError(self, error):
		self._pitchMeasurementError = error

	@property
	def yawError(self):
		return self._yawError

	@yawError.setter
	def yawError(self, error):
		self._yawError = error

	@property
	def yawDriftError(self):
		return self._yawDriftError

	@yawDriftError.setter
	def yawDriftError(self, error):
		self._yawDriftError = error

	@property
	def yawMeasurementError(self):
		return self._yawMeasurementError

	@yawMeasurementError.setter
	def yawMeasurementError(self, error):
		self._yawMeasurementError = error




