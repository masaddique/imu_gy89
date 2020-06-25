import smbus
import numpy
# For gyro


class L3GD20:

    WHO_AM_I = 0X0F
    CTRL_REG1 = 0X20
    CTRL_REG2 = 0X21
    CTRL_REG3 = 0X22
    CTRL_REG4 = 0X23
    CTRL_REG5 = 0X24

    REFERENCE = 0X25
    OUT_TEMP = 0X26
    STATUS_REG = 0X27

    OUT_X_L = 0X28
    OUT_X_H = 0X29
    OUT_Y_L = 0X2A
    OUT_Y_H = 0X2B
    OUT_Z_L = 0X2C
    OUT_Z_H = 0X2D

    FIFO_CTRL_REG = 0X2E
    FIFO_SRC_REG = 0X2F

    INT1_CFG = 0X30
    INT1_SRC = 0X31
    INT1_TSH_XH = 0X32
    INT1_TSH_XL = 0X33
    INT1_TSH_YH = 0X34
    INT1_TSH_YL = 0X35
    INT1_TSH_ZH = 0X36
    INT1_TSH_ZL = 0X37
    INT1_DURATION = 0X38

    GYRO_GAIN_250DPS = 0X00
    GYRO_GAIN_500DPS = 0X10
    GYRO_GAIN_2000DPS = 0X20
    GYRO_UPDATE_CONTINOUS = 0X00
    GYRO_UPDATE_READ = 0X80

    def __init__(self, ad=0x6A, ch=1):
        self.address = ad
        self.channel = 1
        self.bus = smbus.SMBus(ch)
        self.gGain = 8.75/1000

    def init(self, g=GYRO_GAIN_250DPS, u=GYRO_UPDATE_CONTINOUS):
        result = self.bus.read_byte_data(self.address, self.WHO_AM_I)
        if result == 0XD4:
            print("Found l3gd20 on channel 1")
        else:
            print("Unknown device")
            return
        self.bus.write_byte_data(self.address, self.CTRL_REG1, 0X0F)
        r = self.bus.read_byte_data(self.address, self.CTRL_REG4)
        r = r | g | u
        self.bus.write_byte_data(self.address, self.CTRL_REG4, r)
        if g == self.GYRO_GAIN_250DPS:
            self.gGain = 8.75/1000
        elif g == self.GYRO_GAIN_500DPS:
            self.gGain = 17.50/1000
        elif g == self.GYRO_GAIN_2000DPS:
            self.gGain = 70/1000

    def readGyros(self, debug=0):
        # if debug==1:
        #    rd = self.bus.read_byte_data(self.address, self.STATUS_REG);
        #    print("B ",hex(rd));
        xl = self.bus.read_byte_data(self.address, self.OUT_X_L)
        xh = self.bus.read_byte_data(self.address, self.OUT_X_H)
        yl = self.bus.read_byte_data(self.address, self.OUT_Y_L)
        yh = self.bus.read_byte_data(self.address, self.OUT_Y_H)
        zl = self.bus.read_byte_data(self.address, self.OUT_Z_L)
        zh = self.bus.read_byte_data(self.address, self.OUT_Z_H)
        # if debug==1:
        #    rd = self.bus.read_byte_data(self.address, self.STATUS_REG);
        #    print("A ",hex(rd));
        gx = (xh << 8) | xl
        gy = (yh << 8) | yl
        gz = (zh << 8) | zl
        gx = gx-0XFFFE if gx > 0X7FFF else gx
        gy = gy-0XFFFE if gy > 0X7FFF else gy
        gz = gz-0XFFFE if gz > 0X7FFF else gz
        # if (gx>0X7FFF):
        #    gx = gx - 0XFFFE;
        # if (gy>0X7FFF):
        #    gy = gy - 0XFFFE;
        # if (gz>0X7FFF):
        #    gz = gz - 0XFFFE;
        # print(gx,gy,gz);
        #gx = round(gx*self.gGain,3);
        #gy = round(gy*self.gGain,3);
        #gz = round(gz*self.gGain,3);
        return [gx*self.gGain, gy*self.gGain, gz*self.gGain]

    def en_FIFO(self):
        rd = self.bus.read_byte_data(self.address, self.CTRL_REG5)
        self.bus.write_byte_data(self.address, self.CTRL_REG5, rd | 0X40)
        self.bus.write_byte_data(self.address, self.FIFO_CTRL_REG, 0X5E)
        return

    def de_FIFO(self):
        rd = self.bus.read_byte_data(self.address, self.CTRL_REG5)
        self.bus.write_byte_data(self.address, self.CTRL_REG5, rd & 0XBF)
        return

    def read_FIFO(self, debug=0):
        rd = self.bus.read_byte_data(self.address, self.FIFO_SRC_REG)
        if debug == 1:
            print("B: ", hex(rd))
        data = numpy.empty((0, 3), float)
        i = rd & 0X1F
        j = 0
        while j < i:
            k = self.readGyros()
            data = numpy.append(data, numpy.array([k]), axis=0)
            j = j + 1
        if debug == 1:
            rd = self.bus.read_byte_data(self.address, self.FIFO_SRC_REG)
            print("A:", hex(rd))
            print("E")
        return data

    def read_status(self):
        return [self.bus.read_byte_data(self.address, self.STATUS_REG), self.bus.read_byte_data(self.address, self.FIFO_SRC_REG)]
