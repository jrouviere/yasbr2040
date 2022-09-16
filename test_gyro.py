import pimoroni_i2c
import time
import ustruct
import math

def die(code, err):
    print(err)
    while True:
        #TODO: blink a led
        time.sleep(0.5)

 # convert to radian/sec
GYRO_CONV = 1000.0 * math.pi / 180.0 / 32768.0

def init_imu(i2c):
    whoami = i2c.readfrom_mem(0x6A, 0x0F, 1)
    if whoami[0] != 0x6A:
        die(1, f'gyro not connected {whoami}')

    CTRL1_XL = 0x10
    i2c.writeto_mem(0x6A, CTRL1_XL, b'\x60')
    CTRL2_G = 0x11
    i2c.writeto_mem(0x6A, CTRL2_G, b'\x68')
    CTRL7_G = 0x16
    i2c.writeto_mem(0x6A, CTRL7_G, b'\x40')


def read_imu(i2c):
    OUTX_L_G = 0x22
    gyro_x = i2c.readfrom_mem(0x6A, OUTX_L_G, 2) 
    dx = ustruct.unpack("<h", gyro_x)[0] * GYRO_CONV

    OUTY_L_XL = 0x2A
    acc_yz = i2c.readfrom_mem(0x6A, OUTY_L_XL, 4)

    (y,z) = ustruct.unpack("<hh", acc_yz)
    return dx, -math.atan2(z,y)

i2c = pimoroni_i2c.PimoroniI2C(sda=20, scl=21)
init_imu(i2c)

i = 0
theta = 0.0
rate = 1 / 416.0
    
while True:  
    time.sleep(rate)
    
    dt, ang = read_imu(i2c)

    theta = 0.99*(theta+dt*rate) + 0.01*ang
    
    i = (i+1) % 100
    if i == 0:
        print(dt, ang, theta)
