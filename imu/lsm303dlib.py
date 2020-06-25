import smbus
import numpy


class LSM303D:
    # tempurate read out values
    TEMP_OUT_L = 0x05
    TEMP_OUT_H = 0X06

    # magnetometer status and readout
    STATUS_M = 0X07
    OUT_X_L_M = 0X08
    OUT_X_H_M = 0X09
    OUT_Y_L_M = 0X0A
    OUT_Y_H_M = 0X0B
    OUT_Z_L_M = 0X0C
    OUT_Z_H_M = 0X0D

    WHO_AM_I = 0X0F

    INT_CTRL_M = 0X12
    INT_SRC_M = 0X13
    INT_THS_L_M = 0X14
    INT_THS_H_M = 0X15

    OFFSET_X_L_M = 0X16
    OFFSET_X_H_M = 0X17
    OFFSET_Y_L_M = 0X18
    OFFSET_Y_H_M = 0X19
    OFFSET_Z_L_M = 0X1A
    OFFSET_Z_H_M = 0X1B

    # accelerometer reference value for high pass filter
    REFERENCE_X = 0X1C
    REFERENCE_Y = 0X1D
    REFERENCE_Z = 0X1E

    CTRL0 = 0X1F
    CTRL1 = 0X20
    CTRL2 = 0X21
    CTRL3 = 0X22
    CTRL4 = 0X23
    CTRL5 = 0X24
    CTRL6 = 0X25
    CTRL7 = 0X26

    STATUS_A = 0X27
    OUT_X_L_A = 0X28
    OUT_X_H_A = 0X29
    OUT_Y_L_A = 0X2A
    OUT_Y_H_A = 0X2B
    OUT_Z_L_A = 0X2C
    OUT_Z_H_A = 0X2D

    FIFO_CTRL = 0X2E
    FIFO_SRC = 0X2F
    IG_CFG1 = 0X30
    IG_SRC1 = 0X31
    IG_THS1 = 0X32
    IG_DUR1 = 0X33
    IG_CFG2 = 0X34
    IG_SRC2 = 0X35
    IG_THS2 = 0X36
    IG_DUR2 = 0X37

    CLICK_CFG = 0X38
    CLICK_SRC = 0X39
    CLICK_THS = 0X3A

    TIME_LIMIT = 0X3B
    TIME_LATENCY = 0X3C
    TIME_WINDOW = 0X3D
    ACT_THS = 0X3E
    ACT_DUR = 0X3F

    ACCEL_GAIN = 0.000061
    MAG_GAIN = 0.000080

    ACCEL_GAIN_2G = 0.000061
    ACCEL_GAIN_4G = 0.000122
    ACCEL_GAIN_6G = 0.000183
    ACCEL_GAIN_8G = 0.000244
    ACCEL_GAIN_16G = 0.000732

    MAG_GAIN_2G = 0.000080
    MAG_GAIN_4G = 0.000160
    MAG_GAIN_8G = 0.000320
    MAG_GAIN_12G = 0.000479

    ACCEL_GAIN_2M_S2 = 0.000061*9.807
    ACCEL_GAIN_4M_S2 = 0.000122*9.807
    ACCEL_GAIN_6M_S2 = 0.000183*9.807
    ACCEL_GAIN_8M_S2 = 0.000244*9.807
    ACCEL_GAIN_16M_S2 = 0.000732*9.807

    MAG_GAIN_2NT = 0.000080*100000
    MAG_GAIN_4NT = 0.000160*100000
    MAG_GAIN_8NT = 0.000320*100000
    MAG_GAIN_12NT = 0.000479*100000

    ACCEL_RATE_3_125HZ = 0X17
    ACCEL_RATE_6_25HZ = 0X27
    ACCEL_RATE_12_5HZ = 0X37
    ACCEL_RATE_25HZ = 0X47
    ACCEL_RATE_50HZ = 0X57
    ACCEL_RATE_100HZ = 0X67
    ACCEL_RATE_200HZ = 0X77
    ACCEL_RATE_400HZ = 0X87
    ACCEL_RATE_800HZ = 0X97
    ACCEL_RATE_1600HZ = 0XA7

    MAG_RATE_3_125HZ = 0X00
    MAG_RATE_6_25HZ = 0X04
    MAG_RATE_12_5HZ = 0X08
    MAG_RATE_25HZ = 0X0C
    MAG_RATE_50HZ = 0X10
    MAG_RATE_100HZ = 0X14

    MAG_RESOLUTION_LOW = 0X00
    MAG_RESOLUTION_HIGH = 0X60

    def __init__(self, ad=0X1E, ch=1):
        self.address = ad
        self.channel = ch
        self.bus = smbus.SMBus(self.channel)
        self.aGain = self.ACCEL_GAIN_2G
        self.mGain = self.MAG_GAIN_2G

    def init(self):
        self.bus.write_byte_data(self.address, self.CTRL7, 0X00)
        result = self.bus.read_byte_data(self.address, self.WHO_AM_I)
        if result == 0x49:
            print("Found lsm303d on channel 1")
        else:
            print("unknown device")

    def enable_temperature(self):
        rd = self.bus.read_byte_data(self.address, self.CTRL5)
        rd = rd | 0x08
        self.bus.write_byte_data(self.address, self.CTRL5, rd)
        return

    def read_temperature(self):
        rdh = self.bus.read_byte_data(self.address, self.TEMP_OUT_H)
        rdl = self.bus.read_byte_data(self.address, self.TEMP_OUT_L)
        temp = (rdh << 8) + rdl
        if (temp > 0X7FFF):
            temp = temp - 0XFFFE
            return (21+temp/8)

    def enable_magnetometer(self, gain=MAG_GAIN_2G, rate=MAG_RATE_50HZ, resolution=MAG_RESOLUTION_LOW):
        rd = self.bus.read_byte_data(self.address, self.CTRL5)
        rd = rd & 0X80
        rd = rd | rate | resolution
        self.bus.write_byte_data(self.address, self.CTRL5, rd)
        # set scale and gain
        reg = 0X00
        if gain == self.MAG_GAIN_2G:
            reg = 0X00
            self.mGain = gain
        elif gain == self.MAG_GAIN_4G:
            reg = 0X20
            self.mGain = gain
        elif gain == self.MAG_GAIN_8G:
            reg = 0X40
            self.mGain = self.MAG_GAIN_8G
        elif gain == self.MAG_GAIN_12G:
            reg = 0X60
            self.mGain = self.MAG_GAIN_12G
            self.bus.write_byte_data(self.address, self.CTRL6, reg)

    def read_magnetometer(self):
        mxl = self.bus.read_byte_data(self.address, self.OUT_X_L_M)
        mxh = self.bus.read_byte_data(self.address, self.OUT_X_H_M)
        myl = self.bus.read_byte_data(self.address, self.OUT_Y_L_M)
        myh = self.bus.read_byte_data(self.address, self.OUT_Y_H_M)
        mzl = self.bus.read_byte_data(self.address, self.OUT_Z_L_M)
        mzh = self.bus.read_byte_data(self.address, self.OUT_Z_H_M)

        mx = (mxh << 8) + mxl
        my = (myh << 8) + myl
        mz = (mzh << 8) + mzl
        if (mx > 0X7FFF):
            mx = mx - 0XFFFE
        if (my > 0X7FFF):
            my = my - 0XFFFE
        if (mz > 0X7FFF):
            mz = mz - 0XFFFE
            mx = round(mx*self.mGain, 5)
            my = round(my*self.mGain, 5)
            mz = round(mz*self.mGain, 5)
        return [mx, my, mz]

    def enable_accelerometer(self, gain=ACCEL_GAIN_2G, rate=ACCEL_RATE_100HZ):
        #rd = self.bus.read_byte_data(self.address, CTRL1);
        #rd = rd | 0X60;
        self.bus.write_byte_data(self.address, self.CTRL1, rate)
        reg = 0x00
        if gain == self.ACCEL_GAIN_2G:
            self.aGain = gain
            reg = 0X00
        elif gain == self.ACCEL_GAIN_4G:
            self.aGain = gain
            reg = 0X08
        elif gain == self.ACCEL_GAIN_6G:
            self.aGain = gain
            reg = 0X10
        elif gain == self.ACCEL_GAIN_8G:
            self.aGain = gain
            reg = 0X18
        elif gain == self.ACCEL_GAIN_16G:
            self.aGain == gain
            reg = 0X20
            rd = self.bus.read_byte_data(self.address, self.CTRL2)
            rd = rd | reg
            self.bus.write_byte_data(self.address, self.CTRL2, rd)

    def read_accelerometer(self):
        mxl = self.bus.read_byte_data(self.address, self.OUT_X_L_A)
        mxh = self.bus.read_byte_data(self.address, self.OUT_X_H_A)
        myl = self.bus.read_byte_data(self.address, self.OUT_Y_L_A)
        myh = self.bus.read_byte_data(self.address, self.OUT_Y_H_A)
        mzl = self.bus.read_byte_data(self.address, self.OUT_Z_L_A)
        mzh = self.bus.read_byte_data(self.address, self.OUT_Z_H_A)

        mx = (mxh << 8) + mxl
        my = (myh << 8) + myl
        mz = (mzh << 8) + mzl
        if (mx > 0X7FFF):
            mx = mx - 0XFFFE
        if (my > 0X7FFF):
            my = my - 0XFFFE
        if (mz > 0X7FFF):
            mz = mz - 0XFFFE
            mx = round(mx*self.aGain, 5)
            my = round(my*self.aGain, 5)
            mz = round(mz*self.aGain, 5)
        return [mx, my, mz]

    def en_FIFO(self):
        rd = self.bus.read_byte_data(self.address, self.CTRL0)
        self.bus.write_byte_data(self.address, self.CTRL0, rd | 0X40)
        self.bus.write_byte_data(self.address, self.FIFO_CTRL, 0X5E)
        return

    def de_FIFO(self):
        rd = self.bus.read_byte_data(self.address, self.CTRL0)
        self.bus.write_byte_data(self.address, self.CTRL0, rd & 0XBF)
        return

    def read_FIFO(self, debug=0):
        rd = self.bus.read_byte_data(self.address, self.FIFO_SRC)
        if debug == 1:
            print("B: ", hex(rd))
            data = numpy.empty((0, 3), float)
            i = rd & 0X1F
            j = 0
        while j < i:
            k = self.read_accelerometer()
            data = numpy.append(data, numpy.array([k]), axis=0)
            j = j + 1
        if debug == 1:
            rd = self.bus.read_byte_data(self.address, self.FIFO_SRC_REG)
            print("A:", hex(rd))
            print("E")
        return data

    def read_status(self):
        return [self.bus.read_byte_data(self.address, self.STATUS_A), self.bus.read_byte_data(self.address, self.FIFO_SRC)]
