import smbus
import time
import numpy
import math
from os import system, name
from l3gd20lib import L3GD20
from lsm303dlib import LSM303D
from threading import Thread
from fusion import get_position
from fusion import get_velocity
from fusion import get_angles
from fusion import calibrate
from fusion import kalmanFilter

dAddress = 0X1E
dev = LSM303D(dAddress, 1)
dev.init()
dev.enable_temperature()
dev.enable_magnetometer(LSM303D.MAG_GAIN_2G, LSM303D.MAG_RATE_100HZ)
dev.enable_accelerometer(LSM303D.ACCEL_GAIN_2G, LSM303D.ACCEL_RATE_25HZ)

gy = L3GD20(0X6A, 1)
gy.init(L3GD20.GYRO_GAIN_500DPS, L3GD20.GYRO_UPDATE_CONTINOUS)

VarianceMean = calibrate(gy, dev)

kThread = Thread(target=kalmanFilter, args=(gy, dev, VarianceMean))
kThread.start()

while True:
    system("clear")
    print("position: ", get_position())
    print("velocity: ", get_velocity())
    print("angle:", get_angles())

kThread.join()
