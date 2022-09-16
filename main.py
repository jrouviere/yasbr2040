import gc
import time
import math
import ustruct

from motor import Motor, motor2040
from encoder import Encoder, MMME_CPR
from pimoroni import Button, PID, NORMAL_DIR, REVERSED_DIR
import pimoroni_i2c

"""
Code inspired by:
https://github.com/pimoroni/pimoroni-pico/blob/main/micropython/examples/motor2040/position_control.py
"""

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
    i2c.writeto_mem(0x6A, CTRL7_G, b'\x00')


def read_imu(i2c):
    OUTX_L_G = 0x22
    gyro_x = i2c.readfrom_mem(0x6A, OUTX_L_G, 2) 
    dx = ustruct.unpack("<h", gyro_x)[0] * GYRO_CONV

    OUTY_L_XL = 0x2A
    acc_yz = i2c.readfrom_mem(0x6A, OUTY_L_XL, 4)

    (y,z) = ustruct.unpack("<hh", acc_yz)
    return dx, -math.atan2(z,y)

GEAR_RATIO = 20
COUNTS_PER_REV = MMME_CPR * GEAR_RATIO

UPDATES = 416
UPDATE_RATE = 1 / UPDATES

SUB_SAMPLING = 24
PRINT_DIVIDER = 1000

# PID settings

alpha_pid = PID(1.5, 0.0, 0.1, UPDATE_RATE*SUB_SAMPLING)
pos_pid = PID(0.2, 0.0, 0.25, UPDATE_RATE*SUB_SAMPLING)
theta_pid = PID(5.5, 0.10, 0.05, UPDATE_RATE)

gc.collect()

motors = [
    Motor(motor2040.MOTOR_A, freq=21000, deadzone=0.02, direction=NORMAL_DIR),
    Motor(motor2040.MOTOR_D, freq=21000, deadzone=0.02, direction=REVERSED_DIR),
]

encoders = [
    Encoder(0, 0, motor2040.ENCODER_A, direction=REVERSED_DIR, counts_per_rev=COUNTS_PER_REV, count_microsteps=True),
    Encoder(0, 1, motor2040.ENCODER_D, direction=NORMAL_DIR, counts_per_rev=COUNTS_PER_REV, count_microsteps=True),
]

user_sw = Button(motor2040.USER_SW)



i2c = pimoroni_i2c.PimoroniI2C(sda=20, scl=21)
init_imu(i2c)

# Wait for user input to start
while not user_sw.raw():
   time.sleep(0.05)

while user_sw.raw():
    time.sleep(0.2)

for m in motors:
    m.enable()

MAX_TT = 0.12 # maximum angle correction by the position loop
THETA_OFF = -0.26 # stable angle offset

alpha = 0
theta = THETA_OFF
ticks = 0
pos = 0
start = time.ticks_us()

while not user_sw.raw():
    # downsample position pid loop
    if ticks % SUB_SAMPLING == 0:
        # measured linear and rotational movements
        c1 = encoders[0].capture()
        c2 = encoders[1].capture()
        alpha += (c1.delta - c2.delta) / 2000.0
        pos += (c1.delta + c2.delta) / 2000.0
        pos_pid.setpoint = 0.0
        target_theta = -pos_pid.calculate(pos)
        if target_theta < -MAX_TT:
            target_theta = -MAX_TT
        elif target_theta > MAX_TT:
            target_theta = MAX_TT
        alpha_pid.setpoint = 0.0
        alpha_cor = alpha_pid.calculate(alpha)

    
    gyro, acc = read_imu(i2c)
    theta = 0.99*(theta+gyro*UPDATE_RATE) + 0.01*acc
     

    # inclination angle we are aiming for
    #target_theta = 0.0
    theta_pid.setpoint = target_theta+THETA_OFF
    speed_out = theta_pid.calculate(theta)

    if abs(pos)>1.0 or abs(theta)>0.8:
        for m in motors:
            m.disable()
        die(2, "failsafe")

    # Set the new motor driving speed
    motors[0].speed(speed_out + alpha_cor / 2.0)
    motors[1].speed(speed_out - alpha_cor / 2.0)
    


    ticks += 1
        
    now = time.ticks_us()
    delay = (ticks * 1e6 // UPDATES) - time.ticks_diff(now, start)
    time.sleep_us(int(delay))

    if ticks == 10000:
        ticks = 0
        start = time.ticks_us()

    # DBG
    if ticks % PRINT_DIVIDER == 0:
        print("- ALPHA:", alpha, "POS:", pos, "TT:", target_theta, "T:", theta, "delay:", delay)


for m in motors:
    m.disable()
