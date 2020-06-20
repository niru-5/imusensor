from easydict import EasyDict as edict
import numpy as np



def getConfigVals():

	cfg = edict()

	# declaring register values. Don't change them as long as you are damn sure.

	cfg.Address = 0x68

	cfg.AccelOut = 0x3B # getting the accelerometer data.
	cfg.ExtSensData00 = 0x49
	cfg.AccelConfig = 0x1C
	cfg.AccelRangeSelect2G = 0x00
	cfg.AccelRangeSelect4G = 0x08
	cfg.AccelRangeSelect8G = 0x10
	cfg.AccelRangeSelect16G = 0x18

	cfg.GyroConfig = 0x1B
	cfg.GyroRangeSelect250DPS = 0x00
	cfg.GyroRangeSelect500DPS = 0x08
	cfg.GyroRangeSelect1000DPS = 0x10
	cfg.GyroRangeSelect2000DPS = 0x18

	cfg.AccelConfig2 = 0x1D
	cfg.AccelLowPassFilter184 = 0x01
	cfg.AccelLowPassFilter92 = 0x02
	cfg.AccelLowPassFilter41 = 0x03
	cfg.AccelLowPassFilter20 = 0x04
	cfg.AccelLowPassFilter10 = 0x05
	cfg.AccelLowPassFilter5 = 0x06

	cfg.GyroConfig2 = 0x1A
	cfg.GyroLowPassFilter184 = 0x01
	cfg.GyroLowPassFilter92 = 0x02
	cfg.GyroLowPassFilter41 = 0x03
	cfg.GyroLowPassFilter20 = 0x04
	cfg.GyroLowPassFilter10 = 0x05
	cfg.GyroLowPassFilter5 = 0x06

	cfg.SMPDivider = 0x19
	cfg.InitPinConfig = 0x37
	cfg.InitEnable = 0x38
	cfg.InitDisable = 0x00
	cfg.InitPulse50us = 0x00
	cfg.InitWakeOnMotionEnable = 0x40
	cfg.InitRawReadyEnable = 0x01

	cfg.PowerManagement1 = 0x6B
	cfg.PowerCycle = 0x20
	cfg.PowerReset = 0x80
	cfg.ClockPLL = 0x01

	cfg.PowerManagement2 = 0x6C
	cfg.SensorEnable = 0x00
	cfg.DisableGyro = 0x07

	cfg.UserControl = 0x6A
	cfg.I2CMasterEnable = 0x20
	cfg.I2CMasterClock = 0x0D

	cfg.I2CMasterControl = 0x24
	cfg.I2CSlave0Address = 0x25
	cfg.I2CSlave0Register = 0x26
	cfg.I2CSlave0Do = 0x63
	cfg.I2CSlave0Control = 0x27
	cfg.I2CSlave0Enable = 0x80
	cfg.I2CReadFlad = 0x80

	cfg.MotionDetectControl = 0x69
	cfg.AccelIntelEnable = 0x80
	cfg.AccelIntelMode = 0x40

	cfg.LowPassAccelODR = 0x1E
	cfg.LP_ACCEL_ODR_0_24HZ = 0
	cfg.LP_ACCEL_ODR_0_49HZ = 1
	cfg.LP_ACCEL_ODR_0_98HZ = 2
	cfg.LP_ACCEL_ODR_1_95HZ = 3
	cfg.LP_ACCEL_ODR_3_91HZ = 4
	cfg.LP_ACCEL_ODR_7_81HZ = 5
	cfg.LP_ACCEL_ODR_15_63HZ = 6
	cfg.LP_ACCEL_ODR_31_25HZ = 7
	cfg.LP_ACCEL_ODR_62_50HZ = 8
	cfg.LP_ACCEL_ODR_125HZ = 9
	cfg.LP_ACCEL_ODR_250HZ = 10
	cfg.LP_ACCEL_ODR_500HZ = 11

	cfg.WakeOnMotionThreshold = 0x1F
	cfg.WhoAmI = 0x75

	# Ak8963 registers
	cfg.Ak8963I2CAddress = 0x0C
	cfg.Ak8963HXL = 0x03
	cfg.Ak8963CNTL1 = 0x0A
	cfg.Ak8963PowerDown = 0x00
	cfg.Ak8963ContinuosMeasurment1 = 0x12
	cfg.Ak8963ContinuosMeasurment2 = 0x16
	cfg.Ak8963FuseROM = 0x0F
	cfg.Ak8963CNTL2 = 0x0B
	cfg.Ak8963Reset = 0x01
	cfg.Ak8963ASA = 0x10
	cfg.Ak8963WhoAmI = 0x00

	# end of register declaration

	cfg.transformationMatrix = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]).astype(np.int16)
	cfg.I2CRate = 400000
	cfg.TempScale = 333.87
	cfg.TempOffset = 21.0
	cfg.Gravity = 9.807
	cfg.Degree2Radian = np.pi/180.0

	cfg.acc_t_matrix_ned = np.array([[0.0,-1.0,0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]]) #np.array([-1.0,1.0,1.0])
	cfg.gyro_t_matrix_ned = np.array([[0.0,1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,-1.0]]) #np.array([1.0,-1.0,-1.0])
	cfg.mag_t_matrix_ned = np.array([[0.0,-1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,1.0]])

	return cfg



