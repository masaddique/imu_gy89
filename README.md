# imu_gy89
Implementation of position, velocity and orientation using Kalman Filter

TO RUN THE CODE SEE EXAMPLE IN GY89.PY
The parameters are locked to be run on 50 Hz but it is variable and speeds upto 100Hz and more based on gyros and accelerometer parameters.
Regards,
M Ahsan Saddique
email:eemasaddique@gmail.com,mahsan159@live.com

1. fusion.py
a. calibrate(gyroscope, accelerometer)
	This function takes two variables object of classes provided. Function will run for around 20 seconds and readings from gyroscope and accelerometers are taken. These set of values are then used to calculate variance and mean of devices. These readings are used to remove errors from the devices later in running mode. Both accelerometer gyro are required to be still during this calibration phase.

b. kalmanFilter(gyro, accelerometer, a)
	takes same two variables as "calibrate" and output from calibrate function. This is always run in thread mode to get probe for output. However, it can also be used in print mode. 

c. get_position():
	Only works when kalmanFilter is run in thread.

d. get_position():
	Only works when kalmanFilter is run in thread.

e. get_position():
	Only works when kalmanFilter is run in thread.

2. l3gd20lib.py
	contains class L3GD20 to control all gyroscope.
a. def init(GYROGAIN, update)
	Initialize gyroscope takes two inputs:
	i. gyroscope gain
	provided by L3GD20.GYRO_GAIN_250DPS, L3GD20.GYRO_GAIN_500DPS, L3GD20.GYRO_GAIN_2000DPS
	2. update
	provided by L3GD20.GYRO_UPDATE_CONTINOUS, L3GD20.GYRO_UPDATE_READ

b. readGyros()
	will output [x,y,z] of gyroscope as per configurations provided.

c. en_FIFO()
	enable FIFO. FIFO will be enabled in stream mode only i.e. latest 32 values will be taken

d. de_FIFO()
	FIFO disabled

e. read_FIFO()
	gets data from FIFO

f. read_status()
	gets the status of gyroscope in 2X1 arrays. first item will provide gyro status second will provide FIFO status. You can ignore FIFO status if it is not enabled

2. lsm303dlib.py
	contains class LSM303D to control all accelerometer and magnetometer
a. init()
	Initialize accelerometer and magnetometer

b. enable_termperature()
	enable temperature reading. 

c. read_temperature()
	return temperature reading. 

d. enable_magnetometer(gain, rate, resolution)
	takes 3 inputs to enable magnetometer
	i. magnetometer gain
	provided by LSM303D.MAG_GAIN_2G, LSM303D.MAG_GAIN_4G, LSM303D.MAG_GAIN_6G, LSM303D.MAG_GAIN_12G
	ii. update
	provided by LSM303D.MAG_RATE_3_125HZ, LSM303D.MAG_RATE_6_25HZ, LSM303D.MAG_RATE_12_5HZ, LSM303D.MAG_RATE_25HZ,LSM303D.MAG_RATE_50HZ,  LSM303D.MAG_RATE_100HZ,
	iii. provided by LSM303D.MAG_RESOLUTION_LOW, LSM303D.MAG_RESOLUTION_HIGH

e. read_magnetometer()
	will output [x,y,z] of magnetometer as per configurations provided.

f. enable_accelerometer(gain, rate)
	takes 3 inputs to enable magnetometer
	i. magnetometer gain
	provided by LSM303D.ACCEL_GAIN_2G, LSM303D.ACCEL_GAIN_4G, LSM303D.ACCEL_GAIN_6G, LSM303D.ACCEL_GAIN_8G, LSM303D.ACCEL_GAIN_16G
	ii. update
	provided by LSM303D.ACCEL_RATE_3_125HZ, LSM303D.ACCEL_RATE_6_25HZ,  LSM303D.ACCEL_RATE_12_5HZ, LSM303D.ACCEL_RATE_25HZ, LSM303D.ACCEL_RATE_50HZ,  LSM303D.ACCEL_RATE_100HZ, LSM303D.ACCEL_RATE_200HZ, LSM303D.ACCEL_RATE_300HZ, LSM303D.ACCEL_RATE_800HZ, LSM303D.ACCEL_RATE_1600HZ,

g. read_accelerometer()
	will output [x,y,z] of accelerometer as per configurations provided.

c. en_FIFO()
	enable FIFO. FIFO will be enabled in stream mode only i.e. latest 32 values will be taken

d. de_FIFO()
	FIFO disabled

e. read_FIFO()
	gets data from FIFO

f. read_status()
	gets the status of accelerometer and magnetometer in 2X1 arrays. first item will provide device status second will provide FIFO status. You can ignore FIFO status if it is not enabled

