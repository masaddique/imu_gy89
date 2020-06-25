import smbus
import time
import numpy
import math
from os import system, name
from l3gd20lib import L3GD20
from lsm303dlib import LSM303D

out = 0  # this will the variable that will be accessed albiet in parts
x = 0


def calibrate(gyro, accelerometer):
    # this part of routine will only be for calibration process;
    rotationCorrection = [0, 0, 0]
    magBaseCorrection = [0.045833, 0.039468, -0.137264]
    magMatrixCorrection = numpy.array([[1.1039323, -0.04419, -0.000059],
                                       [-0.04419, 1.101589, 0.036200],
                                       [-0.000059, 0.0362, 0.985342]])

    accelBaseCorrection = [0.015928, -0.022339, 0.04299]
    accelMatrixCorrection = numpy.array([[0.971072, 0.006868, 0.000940],
                                         [0.006868, 1.030047, 0.011082],
                                         [0.000940, 0.011082, 1.008003]])
    cAc = numpy.empty((0, 3), float)
    cGy = numpy.empty((0, 3), float)
    cAngles = numpy.empty((0, 3), float)
    cYaw = 0
    cPitch = 0
    cRoll = 0

    calibCount = 1
    print("Calibrating please wait . . . ")

    ts = time.time_ns()
    te = 0
    fm = open("magCalib.txt", "w+")
    fa = open("accCalib.txt", "w+")
    fg = open("gyrCalib.txt", "w+")
    while True:#calibCount < 100:
        calibCount = calibCount + 1
        while (te-ts) <= 200000000:
            te = time.time_ns()

        ts = time.time_ns()
        # print(time.strftime("%H:%M:%S",time.localtime()));
        mRes = numpy.array(accelerometer.read_magnetometer())
        aRes = numpy.array(accelerometer.read_accelerometer())
        grys = gyro.readGyros()
        print(mRes,file=fm)
        print(aRes,file=fa)
        print(numpy.array(grys),file=fg)
        # make correction to magnetic acceleration data due to lattitude and declination
        mRes = mRes-magBaseCorrection
        mResBuf = mRes
        mRes = numpy.dot(magMatrixCorrection, mResBuf.transpose())

        aRes = aRes-accelBaseCorrection
        aResBuf = aRes
        aRes = numpy.dot(accelMatrixCorrection, aResBuf.transpose())
        # calculate the pitch and roll from accelerometer data
        pitch = math.atan2(-aRes[0], math.sqrt(aRes[1]**2 + aRes[2]**2))
        roll = math.atan2(aRes[1], aRes[2])
        cPitch = cPitch + pitch
        cRoll = cRoll + roll
        # calculate the correction to remove gravity component
        rotationCorrection = [-math.sin(pitch), math.sin(roll)
                              * math.cos(pitch), math.cos(roll)*math.cos(pitch)]
        aCorrect = aRes-rotationCorrection

        cAc = numpy.append(cAc, numpy.array([aCorrect]), axis=0)
        # correct magnetometer for yaw
        magXcorrect = mRes[0]*math.cos(pitch) + mRes[1]*math.sin(
            roll)*math.sin(pitch) + mRes[2]*math.cos(roll)*math.sin(pitch)
        magYcorrect = mRes[1]*math.cos(roll) - mRes[2]*math.sin(roll)
        yaw = math.atan2(-magYcorrect, magXcorrect)
        cYaw = cYaw + yaw
        aa = [yaw, pitch, roll]
        cAngles = numpy.append(cAngles, numpy.array([aa]), axis=0)
        cGy = numpy.append(cGy, numpy.array([grys]), axis=0)

    fa.close();
    fm.close();
    fg.close();
    cGyMean = numpy.sum(cGy, axis=0)/calibCount
    cAcMean = numpy.sum(cAc, axis=0)/calibCount
    cYaw = cYaw/calibCount*180/math.pi
    cPitch = cPitch/calibCount*180/math.pi
    cRoll = cRoll/calibCount*180/math.pi
    cAngleMean = [cYaw, cPitch, cRoll]
    #print("Yaw: ", round(cYaw,3)," Roll: ", round(cRoll,3), " Pitch: ", round(cPitch,3));
    #print("Gyro Mean: ", cGyMean);
    #print("Acce Mean: ", cAcMean);
    cGyVar = numpy.sqrt(numpy.sum(numpy.multiply(
        cGy-cGyMean, cGy-cGyMean), axis=0)/calibCount)
    cAcVar = numpy.sqrt(numpy.sum(numpy.multiply(
        cAc-cAcMean, cAc-cAcMean), axis=0)/calibCount)
    cAnglesVar = numpy.sqrt(numpy.sum(numpy.multiply(
        cAngles - cAngleMean, cAngles - cAngleMean), axis=0))/calibCount
    print("Gyro Variance: ", cGyVar)
    print("Acce Variance: ", cAcVar)
    print("Angles Variance: ", cAnglesVar)
    print("Calibration Complete ")
    ret = [cGyVar, cGyMean, cAcVar, cAcMean, cAnglesVar, cAngleMean]
    return ret








