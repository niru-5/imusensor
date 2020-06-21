import numpy as np


class Madgwick:
	"""
	Madgwick filter for sensor fusion of IMU

	The class fuses the roll, pitch and yaw from accelrometer
	and magneotmeter with gyroscope. 
	reference article : https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
	refer to examples of the git repo

	"""
	def __init__(self, b = 0.1):
		"""
		Initialises all the variables. 

		The option of setting your own values is given in the form of 
		set functions

		"""
		
		GyroMeasError = np.pi * (40.0 / 180.0)
		self.beta = np.sqrt(3.0 / 4.0) * GyroMeasError
		# self.beta = b
		self.q = np.array([1.0, 0.0, 0.0, 0.0])
		self.roll = 0
		self.pitch = 0
		self.yaw = 0

	def computeOrientation(self, q):
		"""
		Computes euler angles from quaternion

		Parameter
		---------
		q: array containing quaternion vals

		"""

		self.yaw = np.degrees(np.arctan2(2*q[1]*q[2] + 2*q[0]*q[3],\
							 q[0]*q[0] + q[1]*q[1] - q[2]*q[2] -q[3]*q[3]))
		self.pitch = np.degrees(-1*np.arcsin(2*(q[1]*q[3] - q[0]*q[2])))
		self.roll = np.degrees(np.arctan2(2*q[0]*q[1] + 2*q[2]*q[3],\
								q[0]*q[0] + q[3]*q[3] - q[1]*q[1] - q[2]*q[2]))


	def quaternionMul(self, q1, q2):
		"""
		Provides quaternion multiplication

		Parameters
		----------
		q1: array containing quaternion vals
		q2: array containing quaternion vals

		Return
		------
		finalq: new quaternion obtained from q1*q2
		
		"""
		mat1 = np.array([[0,1,0,0],[-1,0,0,0],[0,0,0,1],[0,0,-1,0]])
		mat2 = np.array([[0,0,1,0],[0,0,0,-1],[-1,0,0,0],[0,1,0,0]])
		mat3 = np.array([[0,0,0,1],[0,0,1,0],[0,-1,0,0],[-1,0,0,0]])

		k1 = np.matmul(q1,mat1)[np.newaxis,:].T
		k2 = np.matmul(q1,mat2)[np.newaxis,:].T
		k3 = np.matmul(q1,mat3)[np.newaxis,:].T
		k0 = q1[np.newaxis,:].T

		mat = np.concatenate((k0,k1,k2,k3), axis = 1)

		finalq = np.matmul(mat,q2)

		return finalq

	def getAccelJacobian(self, q):

		jacob = np.array([[-2.0*q[2], 2.0*q[3], -2.0*q[0], 2.0*q[1]],\
						[2.0*q[1], 2.0*q[0], 2.0*q[3], 2.0*q[2]],\
						[0.0, -4.0*q[1], -4.0*q[2], 0.0]])
		return jacob

	def getAccelFunction(self, q, a):

		func = np.array([[2.0*(q[1]*q[3] - q[0]*q[2]) - a[1]],\
						[2.0*(q[0]*q[1] + q[2]*q[3]) - a[2]],\
						[2.0*(0.5 - q[1]*q[1] - q[2]*q[2]) - a[3]]])
		return func

	def normalizeq(self, q):
		"""
		Normalizing quaternion 

		Parameters
		----------
		q: array containing quaternion vals

		Return
		------
		q: Normalized quaternion
		
		"""

		qLength = np.sqrt(np.sum(np.square(q)))
		q = q/qLength
		return q

	def updateRollAndPitch(self, ax, ay, az, gx, gy, gz, dt):
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

		g = np.array([0.0, gx, gy, gz])
		g = np.radians(g)
		qDot = 0.5*(self.quaternionMul(self.q,g))

		a = np.array([0.0, ax, ay, az])
		a = self.normalizeq(a)

		accelJacob = self.getAccelJacobian(self.q)
		accelF = self.getAccelFunction(self.q, a)

		deltaF = self.normalizeq(np.squeeze(np.matmul(accelJacob.T, accelF)))

		self.q = self.q + (qDot - self.beta*deltaF)*dt
		self.q = self.normalizeq(self.q)
		self.computeOrientation(self.q)

	def getMagJacob(self, q, b):

		magJacob = np.array([[-2*b[3]*q[2], 2*b[3]*q[3], -4*b[1]*q[2] -2*b[3]*q[0], -4*b[1]*q[3] +2*b[3]*q[1] ],\
							[-2*b[1]*q[3] +2*b[3]*q[1], 2*b[1]*q[2] +2*b[3]*q[0], 2*b[1]*q[1] +2*b[3]*q[3], -2*b[1]*q[0] +2*b[3]*q[2]],\
							[2*b[1]*q[2], 2*b[1]*q[3] -4*b[3]*q[1], 2*b[1]*q[0] -4*b[3]*q[2], 2*b[1]*q[1]]])
		return magJacob

	def getMagFunc(self, q, b, m):

		magFunc = np.array([[2*b[1]*(0.5 - q[2]*q[2] - q[3]*q[3]) +2*b[3]*(q[1]*q[3] - q[0]*q[2]) - m[1]],\
							[2*b[1]*(q[1]*q[2] - q[0]*q[3]) +2*b[3]*(q[0]*q[1] + q[2]*q[3]) - m[2]],\
							[2*b[1]*(q[0]*q[2] + q[1]*q[3]) +2*b[3]*(0.5 - q[1]*q[1] -q[2]*q[2]) -m[3]]])
		return magFunc

	def getRotationMat(self, q):

		rotMat = np.array([[2*q[0]*q[0] -1 + 2*q[1]*q[1], 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],\
						[2*(q[1]*q[2] + q[0]*q[3]), 2*q[0]*q[0] -1 + 2*q[2]*q[2], 2*(q[2]*q[3] - q[0]*q[1])],\
						[2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*q[0]*q[0] -1 +2*q[3]*q[3]]])
		return rotMat

	def updateRollPitchYaw(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
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

		g = np.array([0.0, gx, gy, gz])
		g = np.radians(g)
		qDot = 0.5*(self.quaternionMul(self.q,g))

		a = np.array([0.0, ax, ay, az])
		a = self.normalizeq(a)

		accelJacob = self.getAccelJacobian(self.q)
		accelF = self.getAccelFunction(self.q, a)

		m = np.array([0.0, mx, my, mz])
		m = self.normalizeq(m)
		q_rot_mat = self.getRotationMat(self.q)
		h = np.matmul(q_rot_mat,m[1:])
		b = np.array([0.0, 1, 0.0, h[2]])
		b[1] = np.sqrt(np.sum(h[0]*h[0] + h[1]*h[1]))


		magJacob = self.getMagJacob(self.q, b)
		magFunc = self.getMagFunc(self.q, b, m)

		finalJacob = np.concatenate((accelJacob,magJacob), axis=0)
		finalFunc = np.concatenate((accelF, magFunc), axis=0)
		deltaF = self.normalizeq(np.squeeze(np.matmul(finalJacob.T, finalFunc)))

		self.q = self.q + (qDot - self.beta*deltaF)*dt
		self.q = self.normalizeq(self.q)
		self.computeOrientation(self.q)

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
	def beta(self):
		return self._beta

	@beta.setter
	def beta(self, beta):
		if beta >= 0 and beta <= 1:
			self._beta = beta
		else:
			raise Exception("Please put beta value between 0 and 1")

	@property
	def q(self):
		return self._q

	@q.setter
	def q(self, q):
		if q is not None and q.shape[0] == 4:
			self._q = q
		else:
			raise Exception("q has to be a numpy array of 4 elements")
