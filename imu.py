
import math
import ustruct

"""
Helper for ST LSM6DS3 based IMU
"""

class IMU:
    RATE = 416
    # used to convert to radian/sec
    # based on the config use in init
    GYRO_CONV = 1000.0 * math.pi / 180.0 / 32768.0

    OUTX_L_G = 0x22
    OUTX_L_XL = 0x28

    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    CTRL7_G = 0x16

    def __init__(self, i2c):
        self.i2c = i2c

        # check the device is responding
        whoami = i2c.readfrom_mem(0x6A, 0x0F, 1)
        if whoami[0] != 0x6A:
            raise BaseException(f'gyro not connected: {whoami}')

        i2c.writeto_mem(0x6A, self.CTRL1_XL, b'\x60')
        i2c.writeto_mem(0x6A, self.CTRL2_G, b'\x68')
        i2c.writeto_mem(0x6A, self.CTRL7_G, b'\x00')

    def read(self):
        gyro_data = self.i2c.readfrom_mem(0x6A, self.OUTX_L_G, 6)
        gyro = ustruct.unpack("<hhh", gyro_data)
        gyro = [g * self.GYRO_CONV for g in gyro]

        acc_data = self.i2c.readfrom_mem(0x6A, self.OUTX_L_XL, 6)
        acc = ustruct.unpack("<hhh", acc_data)

        return gyro, acc
