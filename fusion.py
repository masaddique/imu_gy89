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
    magBaseCorrection = [0.065943, -0.136893, 0.082006]
    magMatrixCorrection = numpy.array([[1.513754, -0.180472, -0.121769],
                                       [-0.180472, 1.323883, 0.099902],
                                       [-0.121769, 0.099902, 1.357088]])

    accelBaseCorrection = [-0.132742, -0.0044782, 0.212302]
    accelMatrixCorrection = numpy.array([[1.146868, 0.073365, -0.099336],
                                         [0.073365, 1.105934, -0.022481],
                                         [-0.09936, -0.026481, 1.217730]])
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
    while calibCount < 100:
        calibCount = calibCount + 1
        while (te-ts) <= 100000000:
            te = time.time_ns()

        ts = time.time_ns()
        # print(time.strftime("%H:%M:%S",time.localtime()));
        mRes = numpy.array(accelerometer.read_magnetometer())
        aRes = numpy.array(accelerometer.read_accelerometer())
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
        grys = gyro.readGyros()
        cGy = numpy.append(cGy, numpy.array([grys]), axis=0)

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


def kalmanFilter(gyro, accelerometer, a, debug=0):
    global x
    global out
    rotationCorrection = [0, 0, 0]

    magBaseCorrection = [0.065943, -0.136893, 0.082006]
    magMatrixCorrection = numpy.array([[1.513754, -0.180472, -0.121769],
    cGyMean = a[1]
    cGyVar = a[0]
    cAcVar = a[2]
    cAcMean = a[3]
    cAnglesVar = a[4]
    cAngleMean = a[5]

    #angle = [cRoll, cPitch, cYaw];
    timeTT = 20000000  # 20 ms
    te = 1
    # these are the kalman filter vaiables#
    nx = 12
    nz = 6
    nu = 6

    dt = 0.02
    F = numpy.eye(nx)
    G = numpy.zeros(nx*nu)
    G = numpy.resize(G, (nx, nu))
    H = numpy.zeros(nx*nz)
    H = numpy.resize(H, (nz, nx))
    K = numpy.ones(nx*nz)
    K = numpy.resize(K, (nx, nz))
    P = numpy.ones(nx*nx)
    P = numpy.resize(P, (nx, nx))
    Q = numpy.zeros(nx*nx)
    Q = numpy.resize(Q, (nx, nx))
    R = numpy.zeros(nz*nz)
    R = numpy.resize(R, (nz, nz))
    x = numpy.zeros(nx)

    F[3][6] = dt
    F[4][7] = dt
    F[5][8] = dt
    F[3][9] = dt*dt*0.5
    F[4][10] = dt*dt*0.5
    F[5][11] = dt*dt*0.5
    F[6][9] = dt
    F[7][10] = dt
    F[8][11] = dt

    G[0][0] = dt
    G[1][1] = dt
    G[2][2] = dt
    G[3][3] = 0
    G[4][4] = 0
    G[5][5] = 0
    G[6][3] = 0
    G[7][4] = 0
    G[8][5] = 0
    G[9][3] = 1
    G[10][4] = 1
    G[11][5] = 1

    H[0][0] = 1
    H[1][1] = 1
    H[2][2] = 1
    H[3][9] = 1
    H[4][10] = 1
    H[5][11] = 1

    R[0][0] = cAnglesVar[2]
    R[1][1] = cAnglesVar[1]
    R[2][2] = cAnglesVar[0]
    R[3][3] = cAcVar[0]
    R[4][4] = cAcVar[1]
    R[5][5] = cAcVar[2]

    Q[0][0] = 10
    Q[1][1] = 10
    Q[2][2] = 10
    Q[4][4] = 0.01
    Q[5][5] = 0.010
    Q[6][6] = 0.010
    Q[7][7] = 0.010
    Q[6][6] = 0.01
    Q[8][8] = 0.010
    Q[3][3] = 0.010
    Q[9][9] = 0.01
    Q[10][10] = 0.01
    Q[11][11] = 0.01

    # Gyro covariance matrix
    Q[0][0] = 100
    Q[1][1] = 100
    Q[2][2] = 100
    Q[0][1] = 10
    Q[0][2] = 10
    Q[1][0] = 10
    Q[1][2] = 10
    Q[2][0] = 10
    Q[2][1] = 10

    Q[9][0] = 0.01
    Q[10][1] = 0.01
    Q[11][2] = 0.01

    # accelerometer covariance matrix
    Q[3][3] = 0.1
    Q[4][4] = 0.1
    Q[5][5] = 0.1
    Q[6][6] = 0.1
    Q[7][7] = 0.1
    Q[8][8] = 0.1
    Q[9][9] = 0.1
    Q[10][10] = 0.1
    Q[11][11] = 0.1

    # position VOC
    r = 3
    Q[r][4] = 0.01
    Q[r][5] = 0.01
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][9] = 0.0001
    Q[r][10] = 0.0001
    Q[r][11] = 0.0001

    r = 4
    Q[r][3] = 0.01
    Q[r][5] = 0.01
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][9] = 0.0001
    Q[r][10] = 0.0001
    Q[r][11] = 0.0001

    r = 5
    Q[r][3] = 0.01
    Q[r][4] = 0.01
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][9] = 0.0001
    Q[r][10] = 0.0001
    Q[r][11] = 0.0001

    # Velocity COV
    r = 6
    Q[r][3] = 0.001
    Q[r][4] = 0.001
    Q[r][5] = 0.001
    Q[r][7] = 0.01
    Q[r][8] = 0.01
    Q[r][9] = 0.001
    Q[r][10] = 0.001
    Q[r][11] = 0.001

    r = 7
    Q[r][3] = 0.001
    Q[r][4] = 0.001
    Q[r][5] = 0.001
    Q[r][6] = 0.01
    Q[r][8] = 0.01
    Q[r][9] = 0.001
    Q[r][10] = 0.001
    Q[r][11] = 0.001

    r = 8
    Q[r][3] = 0.001
    Q[r][4] = 0.001
    Q[r][5] = 0.001
    Q[r][6] = 0.01
    Q[r][7] = 0.01
    Q[r][9] = 0.001
    Q[r][10] = 0.001
    Q[r][11] = 0.001

    # Acceleration COV
    r = 9
    Q[r][3] = 0.0001
    Q[r][4] = 0.0001
    Q[r][5] = 0.0001
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][10] = 0.01
    Q[r][11] = 0.01

    r = 10
    Q[r][3] = 0.0001
    Q[r][4] = 0.0001
    Q[r][5] = 0.0001
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][9] = 0.01
    Q[r][11] = 0.01

    r = 11
    Q[r][3] = 0.0001
    Q[r][4] = 0.0001
    Q[r][5] = 0.0001
    Q[r][6] = 0.001
    Q[r][7] = 0.001
    Q[r][8] = 0.001
    Q[r][9] = 0.01
    Q[r][10] = 0.01

    Q = Q*1

    ts = 100
    te = 10000

    while True:
        while (te-ts) <= timeTT:
            te = time.time_ns()
        # system("clear");
        #print("Loop Start: ", round((time.time_ns()-ts)/1000000, 3))
        ts = time.time_ns()
        # print(time.strftime("%H:%M:%S",time.localtime()));
        mRes = numpy.array(accelerometer.read_magnetometer())
        aRes = numpy.array(accelerometer.read_accelerometer())
        grys = gyro.readGyros() - cGyMean  # read gyro and remove calibration error

        #print("raw acceleration: ", aRes);
        # make correction to magnetic acceleration data due to lattitude and declination
        mRes = mRes-magBaseCorrection
        mResBuf = mRes
        mRes = numpy.dot(magMatrixCorrection, mResBuf.transpose())

        aRes = aRes - accelBaseCorrection
        aResBuf = aRes
        aRes = numpy.dot(accelMatrixCorrection, aResBuf.transpose())

        # calculate the pitch and roll from accelerometer data
        pitch = math.atan2(-aRes[0], math.sqrt(aRes[1]**2 + aRes[2]**2))
        roll = math.atan2(aRes[1], aRes[2])
        # calculate the correction to remove gravity component
        rotationCorrection = [-math.sin(pitch), math.sin(roll)
                              * math.cos(pitch), math.cos(roll)*math.cos(pitch)]
        aCorrect = aRes-rotationCorrection-cAcMean
        # correct magnetometer for yaw
        magXcorrect = mRes[0]*math.cos(pitch) + mRes[1]*math.sin(
            roll)*math.sin(pitch) + mRes[2]*math.cos(roll)*math.sin(pitch)
        magYcorrect = mRes[1]*math.cos(roll) - mRes[2]*math.sin(roll)
        yaw = math.atan2(-magYcorrect, magXcorrect)
        angle = [roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi]

        # this part of code will implement kalman filter

        u = numpy.hstack((grys, 9.796*aCorrect))
        x = numpy.dot(F, x) + numpy.dot(G, u)
        P = numpy.dot(numpy.dot(F, P), F.transpose()) + Q
        a = numpy.hstack((angle, numpy.zeros(6), 9.769*aCorrect))
        z = numpy.dot(H, a)
        K1 = numpy.dot(P, H.transpose())
        K2 = numpy.dot(numpy.dot(H, P), H.transpose())
        K3 = numpy.linalg.inv(K2 + R)
        K = numpy.dot(K1, K3)

        x = x + numpy.dot(K, z - numpy.dot(H, x))
        P1 = numpy.eye(nx) - numpy.dot(K, H)
        P2 = numpy.dot(numpy.dot(K, R), K.transpose())
        P = numpy.dot(numpy.dot(P1, P), P1.transpose()) + P2
        out = x
        if debug == 1:
            print('Angle:', x[0:3])
            print('Velocity:', x[6:9])
            print('Position:', x[3:6])


def get_position():
    return out[3:6]


def get_velocity():
    return out[6:9]


def get_angles():
    return out[0:3]
