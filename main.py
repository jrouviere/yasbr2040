import gc
import time
import math
import sys

from imu import IMU
from ibus_rx import IBus
from robot import Robot

from motor import Motor, motor2040
from encoder import Encoder
from pimoroni import Button, NORMAL_DIR, REVERSED_DIR

from machine import I2C, UART, Pin


def main():
    motors = [
        Motor(motor2040.MOTOR_A, freq=21000, deadzone=0.02, direction=NORMAL_DIR),
        Motor(motor2040.MOTOR_D, freq=21000, deadzone=0.02, direction=REVERSED_DIR),
    ]

    encoders = [
        Encoder(0, 0, motor2040.ENCODER_A, direction=REVERSED_DIR, count_microsteps=True),
        Encoder(0, 1, motor2040.ENCODER_D, direction=NORMAL_DIR, count_microsteps=True),
    ]

    i2c = I2C(0, sda=Pin(20), scl=Pin(21))
    imu = IMU(i2c)

    uart0 = UART(0, tx=Pin(16), rx=Pin(17))
    ibus = IBus(uart0)

    user_sw = Button(motor2040.USER_SW)
    wait_btn(user_sw)

    for m in motors:
        m.enable()
    
    robot = Robot(imu, ibus, motors, encoders)
    
    try:
        robot.run()
    except BaseException as e:
        sys.print_exception(e)
    finally:        
        for m in motors:
            m.disable()


def wait_btn(user_btn):
    # Wait for user to press and release the button
    while not user_btn.raw():
        time.sleep_us(5000)

    while user_btn.raw():
        time.sleep_us(100000)


if __name__ == "__main__":
    gc.collect()
    main()
