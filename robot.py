import math
import time

from machine import Timer
from pimoroni import PID
from imu import IMU
from ibus_rx import IBus

class Robot:
    ODO_PERIOD = 60 # in milliseconds
    PRINT_DIVIDER = 1000
    MAX_TT = 0.12 # maximum angle correction by the position loop
    THETA_OFF = -0.26 # stable angle offset


    def __init__(self, imu, ibus, motors, encoders):
        assert len(motors) == 2
        assert len(encoders) == 2

        self.imu = imu
        self.ibus = ibus
        self.motors = motors
        self.encoders = encoders

        self.UPDATE_PER = 1 / imu.RATE
        # PID settings
        self.alpha_pid  = PID(1.50, 0.0, 0.10, self.ODO_PERIOD/1000)
        self.pos_pid    = PID(0.20, 0.0, 0.25, self.ODO_PERIOD/1000)
        self.theta_pid  = PID(5.50, 0.1, 0.05, self.UPDATE_PER)
        
        # pos and angles
        self.alpha = 0.0
        self.pos = 0.0
        self.theta = self.THETA_OFF

        # controls
        self.target_theta = 0.0
        self.alpha_cor = 0.0


    def update_imu(self):
        gyro, acc = self.imu.read()
        dt = gyro[0]
        ang = -math.atan2(acc[2], acc[1])
        # fuse accelerometer and gyroscope mesurements
        self.theta = 0.99*(self.theta+dt*self.UPDATE_PER) + 0.01*ang


    def update_odometry(self, t):
        # measured linear and rotational movements
        c1 = self.encoders[0].capture()
        c2 = self.encoders[1].capture()
        self.alpha += (c1.delta - c2.delta) / 2000.0 
        self.pos += (c1.delta + c2.delta) / 2000.0
        self.pos_pid.setpoint = 0.0

        self.target_theta = clamp(-self.pos_pid.calculate(self.pos), self.MAX_TT)

        self.alpha_pid.setpoint = 0.0
        self.alpha_cor = self.alpha_pid.calculate(self.alpha)


    def log_debug(self, t):
        print("- ALPHA:", self.alpha, "POS:", self.pos, "TT:", self.target_theta, "T:", self.theta, "delay:", self.delay)


    def run(self):
        self.odo_timer = Timer(period=self.ODO_PERIOD, mode=Timer.PERIODIC, callback=self.update_odometry)
        self.dbg_timer = Timer(period=1000, mode=Timer.PERIODIC, callback=self.log_debug)

        ticks = 0
        start = time.ticks_us()

        while True:
            self.update_imu()

            # inclination angle we are aiming for
            self.theta_pid.setpoint = self.target_theta+self.THETA_OFF
            speed_out = self.theta_pid.calculate(self.theta)

            # Set the new motor driving speed
            self.motors[0].speed(speed_out + self.alpha_cor / 2.0)
            self.motors[1].speed(speed_out - self.alpha_cor / 2.0)

            # failsafe
            if abs(self.pos)>1.0 or abs(self.theta)>0.8:
                raise BaseException("failsafe")

            # timer management
            ticks += 1
            now = time.ticks_us()
            self.delay = (ticks * 1e6 // self.imu.RATE) - time.ticks_diff(now, start)
            time.sleep_us(int(self.delay))


def clamp(val, vmax):
    if val < -vmax:
        return -vmax
    if val > vmax:
        return vmax
    return val
